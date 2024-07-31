/*********************************************************************
 * @file trajopt_node.cu
 
 * @brief TrajoptNode implementation.
 *********************************************************************/
#include <filesystem>
#include <tuple>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "settings.cuh"
#include "dynamics/rbd_plant.cuh"
#include "gpu_pcg.cuh"
#include "pcg/sqp.cuh"
#include "utils/experiment.cuh"
#include "mpc_settings.hpp"

/**
 * @brief TrajoptNode class.
 *
 * MPC Controller for Indy7 robot using Trajopt. Subscribes to current state,  publishes optimized joint position trajectory.
 *
 * @param goal_eePos_traj_2d 2D trajectory of goal end effector positions
 */
class TrajoptNode : public rclcpp::Node
{
public:
    TrajoptNode(const std::vector<std::vector<float>>& goal_eePos_traj_2d)
     : Node("trajopt_node"), stop_trajopt_(false), state_updated_(false), current_time_(0.0), last_time_(0.0), traj_offset_(0), goal_traj_length_(goal_eePos_traj_2d.size()),
       timestep_(TIMESTEP), pcg_exit_tol_(5e-6), pcg_max_iter_(173), traj_length_((STATE_SIZE + CONTROL_SIZE) * KNOT_POINTS - CONTROL_SIZE)
    {
         RCLCPP_INFO(this->get_logger(), "Initializing TrajoptNode");

        initializeTrajopt(goal_eePos_traj_2d);

        //buffer of 1 for most recent state
        state_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "curr_state_and_time", 1, std::bind(&TrajoptNode::stateCallback, this, std::placeholders::_1));
        
        traj_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_pos_traj", 1);

        RCLCPP_INFO(this->get_logger(), "Waiting for initial state...");
        while (rclcpp::ok() && !state_updated_) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        RCLCPP_INFO(this->get_logger(), "Initial state received.");

        warmStartTrajopt();
        trajopt_thread_ = std::thread(&TrajoptNode::trajoptLoop, this);
    }

    ~TrajoptNode()
    {
        stop_trajopt_ = true; // stop trajopt thread
        if (trajopt_thread_.joinable()) {
            trajopt_thread_.join();
        }
        cleanupTrajopt();
    }

private:
    void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.assign(msg->data.begin(), msg->data.begin() + 6);
        current_time_ = msg->data.back();
        RCLCPP_INFO(this->get_logger(), "<stateCallback()>: Received state: %f, %f, %f, %f, %f, %f at time %f", current_state_[0], current_state_[1], current_state_[2], current_state_[3], current_state_[4], current_state_[5], current_time_);
        state_updated_ = true;
    }

    void initializeTrajopt(const std::vector<std::vector<float>>& goal_eePos_traj_2d)
    {
        checkPcgOccupancy<float>((void *) pcg<float, STATE_SIZE, KNOT_POINTS>, PCG_NUM_THREADS, STATE_SIZE, KNOT_POINTS);
        pcg_config_.pcg_block = PCG_NUM_THREADS; //128
        pcg_config_.pcg_exit_tol = pcg_exit_tol_;
        pcg_config_.pcg_max_iter = pcg_max_iter_;    
        
        current_state_.resize(6);

        // Populate 1D vector with 2D input goal trajectory
        std::vector<float> h_goal_eePos_traj;
        for (const auto& vec : goal_eePos_traj_2d) { h_goal_eePos_traj.insert(h_goal_eePos_traj.end(), vec.begin(), vec.end()); }

        // Initialize device memory
        gpuErrchk(cudaMalloc(&d_goal_eePos_traj_, h_goal_eePos_traj.size() * sizeof(float))); //full goal trajectory
        gpuErrchk(cudaMemcpy(d_goal_eePos_traj_, h_goal_eePos_traj.data(), h_goal_eePos_traj.size() * sizeof(float), cudaMemcpyHostToDevice));
        gpuErrchk(cudaMalloc(&d_eePos_traj_, 6 * KNOT_POINTS * sizeof(float))); //trajectory for trajopt
        gpuErrchk(cudaMemcpy(d_eePos_traj_, d_goal_eePos_traj_, 6 * KNOT_POINTS * sizeof(float), cudaMemcpyDeviceToDevice)); //initialize start of trajectory
        gpuErrchk(cudaMalloc(&d_xu_, traj_length_ * sizeof(float))); //initialized later with current state in warmStartTrajopt()
        gpuErrchk(cudaMalloc(&d_lambda_, STATE_SIZE * KNOT_POINTS * sizeof(float)));
        gpuErrchk(cudaMemset(d_lambda_, 0, STATE_SIZE * KNOT_POINTS * sizeof(float)));
        d_dynmem_const_ = gato_plant::initializeDynamicsConstMem<float>(); //TODO: import this from MPCGPU/include/common/integrator.cuh
    }

    void cleanupTrajopt()
    {
        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaFree(d_xu_));
        gpuErrchk(cudaFree(d_eePos_traj_));
        gpuErrchk(cudaFree(d_lambda_));
        gpuErrchk(cudaFree(d_goal_eePos_traj_));
        gato_plant::freeDynamicsConstMem<float>(d_dynmem_const_);
    }

    void warmStartTrajopt()
    {        
        // get current state
        std::vector<float> current_state;
        {
            //std::lock_guard<std::mutex> lock(state_mutex_);
            current_state = current_state_;
            RCLCPP_INFO(this->get_logger(), "<warmStartTrajopt()>: Warm starting trajopt..");
        }

        // copy current state to init host xu vector
        h_xu_.resize(traj_length_);
        for (int i = 0; i < KNOT_POINTS; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (i < KNOT_POINTS - 1) //not final knot point
                {
                    h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + j] = current_state[j]; // set current state
                    h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 6 + j] = 0.0; // set velocities to 0
                    h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 12 + j] = 0.0; // set torques to 0
                }
                else // final knot point, only set current state, no control
                {
                    h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + j] = current_state[j]; 
                    h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 6 + j] = 0.0;
                }
            }
        }
        
        // copy xu to device
        gpuErrchk(cudaMemcpy(d_xu_, h_xu_.data(), traj_length_ * sizeof(float), cudaMemcpyHostToDevice));

        //remove jitters TODO: test if this is necessary
        pcg_config_.pcg_exit_tol = 1e-11;
        pcg_config_.pcg_max_iter = 10000;
        for(int j = 0; j < 100; j++)
        {
            sqpSolvePcg<float>(STATE_SIZE, CONTROL_SIZE, KNOT_POINTS, timestep_, d_eePos_traj_, d_lambda_, d_xu_, d_dynmem_const_, pcg_config_, rho_, 1e-3);
            gpuErrchk(cudaMemcpy(d_xu_, h_xu_.data(), traj_length_ * sizeof(float), cudaMemcpyHostToDevice));
        }
        //reset pcg config
        rho_ = 1e-3;
        pcg_config_.pcg_exit_tol = pcg_exit_tol_;
        pcg_config_.pcg_max_iter = pcg_max_iter_;

        RCLCPP_INFO(this->get_logger(), "<warmStartTrajopt()>: Done warm starting trajopt");
    }

    void trajoptLoop()
    {
        rclcpp::Rate rate(200); //  TODO: tweak this/make a parameter

        while (rclcpp::ok() && !stop_trajopt_ && traj_offset_ < goal_traj_length_) //until end of goal trajectory
        {
            if (updateDeviceState()) //check if we have a new state, then update and run trajopt
            {
                runTrajoptIteration();
                publishTraj();
            }
            rate.sleep();
        }
        RCLCPP_INFO(this->get_logger(), "---------------------------------\n\n");
        RCLCPP_INFO(this->get_logger(), "<trajoptLoop()>: Trajopt loop finished");
        exit(0);
    }

    bool updateDeviceState()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_updated_)
        {
            if (last_time_ == 0.0) { last_time_ = current_time_; } //initialize last_time_
            RCLCPP_INFO(this->get_logger(), "---------------------------------");
            while (last_time_ + timestep_ < current_time_)
            {
                traj_offset_++;
                last_time_ += timestep_;
                
                RCLCPP_INFO(this->get_logger(), "<updateDeviceState()>: Shifting to trajectory offset: %d", traj_offset_);
                // shift XU over by one knot point
                just_shift<float>(STATE_SIZE, CONTROL_SIZE, KNOT_POINTS, d_xu_);
                //fill in x_N = x_N-1, set u_N-1 to 0
                gpuErrchk(cudaMemcpy(&d_xu_[traj_length_ - STATE_SIZE], &d_xu_[traj_length_-(2*STATE_SIZE)-CONTROL_SIZE], STATE_SIZE * sizeof(float), cudaMemcpyDeviceToDevice));
                gpuErrchk(cudaMemset(&d_xu_[traj_length_ - (STATE_SIZE + CONTROL_SIZE)], 0, CONTROL_SIZE * sizeof(float)));

                //shift eePos_traj over by one knot point
                just_shift<float>(6, 0, KNOT_POINTS, d_eePos_traj_);

                //copy d_eePos_traj_ to host
                std::vector<float> h_eePos_traj(6 * KNOT_POINTS);
                gpuErrchk(cudaMemcpy(h_eePos_traj.data(), d_eePos_traj_, 6 * KNOT_POINTS * sizeof(float), cudaMemcpyDeviceToHost));

                if (traj_offset_ + KNOT_POINTS < goal_traj_length_) // fill in last knot with next goal position from input
                {
                    gpuErrchk(cudaMemcpy(&d_eePos_traj_[(KNOT_POINTS - 1) * 6], &d_goal_eePos_traj_[(traj_offset_ + KNOT_POINTS - 1) * 6], 6 * sizeof(float), cudaMemcpyDeviceToDevice));
                }
                else // if close to end of trajectory, fill in last knot point with goal position
                {
                    gpuErrchk(cudaMemcpy(&d_eePos_traj_[(KNOT_POINTS - 1) * 6], &d_goal_eePos_traj_[(goal_traj_length_ - 1) * 6], 6 * sizeof(float), cudaMemcpyDeviceToDevice));
                }
                
                just_shift<float>(STATE_SIZE, 0, KNOT_POINTS, d_lambda_); //shift lambdas
            }
            gpuErrchk(cudaMemcpy(d_xu_, current_state_.data(), STATE_SIZE * sizeof(float), cudaMemcpyHostToDevice)); // set x_0 to current state
            current_trajopt_time_ = current_time_;
            state_updated_ = false; 

            return true; //true to run trajopt iteration
        }
        return false;
    }

    void runTrajoptIteration()
    {
        sqp_stats = sqpSolvePcg<float>(STATE_SIZE, CONTROL_SIZE, KNOT_POINTS, timestep_, d_eePos_traj_, d_lambda_, d_xu_, d_dynmem_const_, pcg_config_, rho_, rho_reset_);

        auto [pcg_iters, linsys_times, sqp_solve_time, sqp_iters, sqp_exit, pcg_exits] = sqp_stats;
        RCLCPP_INFO(this->get_logger(), "<runTrajoptIteration()>: Trajopt stats: SQP iters: %d, Time: %.3f us, Exit status: %s", sqp_iters, sqp_solve_time, sqp_exit ? "Success" : "Failure");   
    }

    void publishTraj()
    {
        auto traj_msg = std_msgs::msg::Float64MultiArray();
        
        //copy optimized trajectory to host
        gpuErrchk(cudaMemcpy(h_xu_.data(), d_xu_ , traj_length_ * sizeof(float), cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize()); //TODO: is this necessary?

        RCLCPP_INFO(this->get_logger(), "<publishTraj()>: Publishing trajectory @ start time: %f", current_trajopt_time_);

        //populate traj_msg with optimized trajectory
        for (int i = 0; i<KNOT_POINTS; i++)
        {
            traj_msg.data.push_back(h_xu_[i*(STATE_SIZE + CONTROL_SIZE)]);
            traj_msg.data.push_back(h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 1]);
            traj_msg.data.push_back(h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 2]);
            traj_msg.data.push_back(h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 3]);
            traj_msg.data.push_back(h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 4]);
            traj_msg.data.push_back(h_xu_[i*(STATE_SIZE + CONTROL_SIZE) + 5]);
        }
        traj_msg.data.push_back(current_trajopt_time_);
        traj_publisher_->publish(traj_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_subscription_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_publisher_;

    const uint32_t traj_length_; //length of xu trajectory
    const uint32_t goal_traj_length_; //length of full goal trajectory
    const float timestep_;
    const float pcg_exit_tol_;
    const int pcg_max_iter_;
    uint32_t traj_offset_; //offset of current traj in full goal trajectory
    std::vector<float> current_state_; //current state (updated by stateCallback) TODO: add velocities
    double current_time_; //current time (updated by stateCallback)
    std::atomic<bool> state_updated_; //flag to check if state has been updated
    std::vector<float> h_xu_; //host xu trajectory
    double current_trajopt_time_; //current time of trajopt iteration, sent with published trajectory as x_0
    double last_time_; // time of last trajopt iteration
    float *d_goal_eePos_traj_; //full goal trajectory
    float *d_eePos_traj_, *d_xu_, *d_lambda_; 
    void *d_dynmem_const_; //dynamics constant memory
    pcg_config<float> pcg_config_;
    float rho_ = 1e-3;
    float rho_reset_ = 1e-3;

    std::tuple<std::vector<int>, std::vector<double>, double, uint32_t, bool, std::vector<bool>> sqp_stats; //return from sqpSolvePcg

    std::mutex state_mutex_;

    std::thread trajopt_thread_;
    std::atomic<bool> stop_trajopt_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    char eePos_traj_file_name[100] = "trajectories/figure8_traj_eePos_meters.csv";
	std::vector<std::vector<float>> goal_eePos_traj_2d = readCSVToVecVec<float>(eePos_traj_file_name);
    auto node = std::make_shared<TrajoptNode>(goal_eePos_traj_2d);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}