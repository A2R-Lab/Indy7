#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sqp.cuh"  // Include CUDA header
#include <filesystem>

class TrajoptNode : public rclcpp::Node
{
public:
    TrajoptNode(const std::vector<std::vector<float>>& goal_eePos_traj_2d)
     : Node("mpc_controller"), stop_trajopt_(false), current_time_(0.0), last_time_(0.0),
       traj_offset_(0)
    {
        initializeTrajopt(goal_eePos_traj_2d);

        //buffer of 1 for most recent state
        state_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "curr_state_and_time", 1, std::bind(&TrajoptNode::stateCallback, this, std::placeholders::_1));
        
        traj_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("robot_control", 1);

        warmStartTrajopt();

        trajopt_thread_ = std::thread(&TrajoptNode::trajoptLoop, this);
    }

    ~TrajoptNode()
    {
        stop_trajopt_ = true; // stop trajopt thread
        trajopt_thread_.join();
        cleanupTrajopt();
    }

private:
    void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        current_state_.assign(msg->data.begin(), msg->data.begin() + 6);
        current_time_ = msg->data.back();
        RCLCPP_INFO(this->get_logger(), "Received state: %f, %f, %f, %f, %f, %f at time %f", current_state_[0], current_state_[1], current_state_[2], current_state_[3], current_state_[4], current_state_[5], current_time_);
        state_updated_ = true;
    }

    void initializeTrajopt(const std::vector<std::vector<float>>& goal_eePos_traj_2d)
    {
        traj_length_ = (STATE_SIZE + CONTROL_SIZE) * KNOT_POINTS - CONTROL_SIZE;
        
        const std::vector<float> goal_eePos_traj; //1D vector of eePos_traj
        for (const auto& vec : goal_eePos_traj_2d)
        {
            goal_eePos_traj.insert(goal_eePos_traj.end(), vec.begin(), vec.end());
        }

        goal_traj_length_ = goal_eePos_traj.size();

        gpuErrchk(cudaMalloc(&d_goal_eePos_traj_, goal_eePos_traj.size() * sizeof(float)));
        gpuErrchk(cudaMemcpy(d_goal_eePos_traj_, goal_eePos_traj.data(), goal_eePos_traj.size() * sizeof(float), cudaMemcpyHostToDevice));
        gpuErrchk(cudaMalloc(&d_xu_, traj_length_ * sizeof(float)));
        gpuErrchk(cudaMalloc(&d_eePos_traj_, 6 * KNOT_POINTS * sizeof(float)));
        gpuErrchk(cudaMemcpy(d_eePos_traj_, d_goal_eePos_traj_, 6 * KNOT_POINTS * sizeof(float), cudaMemcpyHostToDevice)); // initialize with start of trajectory
        gpuErrchk(cudaMalloc(&d_lambda_, STATE_SIZE * KNOT_POINTS * sizeof(float)));
        gpuErrchk(cudaMemset(d_lambda_, 0, STATE_SIZE * KNOT_POINTS * sizeof(float)));
        
        d_dynmem_const_ = gato_plant::initializeDynamicsConstMem<T>(); //TODO: import this from MPCGPU/include/common/integrator.cuh

        config_.pcg_block = PCG_NUM_THREADS; //128
        config_.pcg_exit_tol = LINSYS_EXIT_TOL;
        config_.pcg_max_iter = PCG_MAX_ITER; //173 (?)

        current_state_.resize(6); //TODO: change to torques
        state_updated_ = false;
    }

    void cleanupTrajopt()
    {
        gpuErrchk(cudaDeviceSynchronize());
        gpuErrchk(cudaFree(d_xu_));
        gpuErrchk(cudaFree(d_eePos_traj_));
        gpuErrchk(cudaFree(d_lambda_));
        gpuErrchk(cudaFree(d_goal_eePos_traj_));
    }

    void warmStartTrajopt()
    {
        // get current state. if no state, raise error
        double current_time;
        std:vector<float> current_state(6);
        std::vector<float> h_xu(traj_length_); // host xu vector

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state = current_state_;
            current_time = current_time_;
        }

        if (current_time == 0.0) // current state hasn't been updated yet
        {
            RCLCPP_ERROR(this->get_logger(), "Current state is not set (current_time is 0.0). Cannot warm start trajopt.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Warm starting trajopt with state: %f, %f, %f, %f, %f, %f", current_state[0], current_state[1], current_state[2], current_state[3], current_state[4], current_state[5]);

        // copy current state to init host xu vector
        for (int i = 0; i < KNOT_POINTS; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (i < KNOT_POINTS - 1)
                {
                    h_xu[i*(STATE_SIZE + CONTROL_SIZE) + j] = current_state[j]; // set current state
                    h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 6 + j] = 0.0; // set velocities to 0
                    h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 12 + j] = 0.0; // set torques to 0

                }
                else
                {
                    h_xu[i*(STATE_SIZE + CONTROL_SIZE) + j] = current_state[j]; // set final state
                    h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 6 + j] = 0.0;
                }

            }
        }
        RCLCPP_INFO(this->get_logger(), "h_xu: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f ...", h_xu[0], h_xu[1], h_xu[2], h_xu[3], h_xu[4], h_xu[5], h_xu[6], h_xu[7], h_xu[8], h_xu[9], h_xu[10], h_xu[11], h_xu[12], h_xu[13]);

        // copy host xu vector to device xu vector
        gpuErrchk(cudaMemcpy(d_xu_, h_xu.data(), traj_length_ * sizeof(float), cudaMemcpyHostToDevice));

        config_.pcg_exit_tol = 1e-11;
        config_.pcg_max_iter = 10000;

        for(int j = 0; j < 100; j++)
        {
            sqpSolvePcg<float>(STATE_SIZE, CONTROL_SIZE, KNOT_POINTS, TIMESTEP, d_eePos_traj_, d_lambda_, d_xu_, d_dynMem_const_, config_, rho_, 1e-3);
            gpuErrchk(cudaMemcpy(d_xu_, h_xu.data(), traj_length_ * sizeof(float), cudaMemcpyHostToDevice));
        }
        rho = 1e-3;
        config.pcg_exit_tol = LINSYS_EXIT_TOL;
        config.pcg_max_iter = PCG_MAX_ITER; //TODO: make this a parameter

        RCLCPP_INFO(this->get_logger(), "Done warm starting trajopt");
    }

    void trajoptLoop()
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_time_ = current_time_;
        }
        while (!stop_trajopt_)
        {
            if (updateDeviceState()) //check if we have a new state
            {
                runTrajoptIteration();
                publishTraj(); //publish optimized trajectory to ROS
            }
            else
            {
                // No new state, sleep a bit to avoid busy waiting
                std::this_thread::sleep_for(std::chrono::milliseconds(1)); //TODO: tweak this
            }
        }
    }

    bool updateDeviceState()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (state_updated_)
        {
            for (double time_diff=current_time_-last_time_; time_diff > TIMESTEP; time_diff -= TIMESTEP)
            {
                traj_offset_++;
                last_time_ += TIMESTEP;
                // shift xu
                just_shift<T>(STATE_SIZE, CONTROL_SIZE, KNOT_POINTS, d_xu_);
                //fill in x_N = x_N-1, and set u_N-1 to 0
                gpuErrchk(cudaMemcpy(d_xu_[traj_length_ - STATE_SIZE], d_xu_[traj_length_-2*STATE_SIZE-CONTROL_SIZE], STATE_SIZE * sizeof(float), cudaMemcpyDeviceToDevice));
                gpuErrchk(cudaMemset(d_xu_[traj_length_ - (STATE_SIZE + CONTROL_SIZE)], 0, CONTROL_SIZE * sizeof(float)));

                //shift eePos_traj
                just_shift(6, 0, KNOT_POINTS, d_eePos_traj_);
                if (traj_offset_ + KNOT_POINTS < goal_traj_length_)
                {
                    gpuErrchk(cudaMemcpy(d_eePos_traj_ + (KNOT_POINTS - 1) * 6, d_goal_eePos_traj_ + (traj_offset_ + KNOT_POINTS - 1) * 6, 6 * sizeof(float), cudaMemcpyDeviceToDevice));
                }
                else // if close to end of trajectory, fill in last state with goal position
                {
                    gpuErrchk(cudaMemcpy(d_eePos_traj_ + (KNOT_POINTS - 1) * 6, d_goal_eePos_traj_ + (goal_traj_length_ - 1) * 6, 6 * sizeof(float), cudaMemcpyDeviceToDevice));
                }
                //shift lambdas
                just_shift(STATE_SIZE, 0, KNOT_POINTS, d_eePos_traj_);
            }
            // TODO: get velocities maybe?
            gpuErrchk(cudaMemcpy(d_xu_, current_state_.data(), STATE_SIZE * sizeof(float), cudaMemcpyHostToDevice));
            current_trajopt_time_ = current_time_;
            state_updated_ = false;
            return true;
        }
        return false;
    }

    void runTrajoptIteration()
    {
        sqp_stats = sqpSolvePcg<float>(STATE_SIZE, CONTROL_SIZE, KNOT_POINTS, TIMESTEP, d_eePos_traj_, d_lambda_, d_xu_, d_dynMem_const_, config_, rho_, rho_reset_);

        auto [pcg_iters, linsys_times, sqp_solve_time, sqp_iters, sqp_exit, pcg_exits] = sqp_stats;

        RCLCPP_INFO(this->get_logger(), "Trajopt iteration: SQP iters: %d, Time: %.2f ms, PCG iters: %d, Exit status: %s",
                    sqp_iters, sqp_solve_time, pcg_iters.back(), sqp_exit ? "Success" : "Failure");   
    }

    void publishTraj()
    {
        std::vector<float> h_xu(traj_length_);
        auto traj_msg = std_msgs::msg::Float64MultiArray();
        traj_msg.data.resize(6*KNOT_POINTS);
        
        gpuErrchk(cudaMemcpy(h_xu.data(), d_xu_ , traj_length * sizeof(float), cudaMemcpyDeviceToHost));
        gpuErrchk(cudaDeviceSynchronize()); //TODO: is this necessary?

        for (int i = 0; i<KNOT_POINTS; i++)
        {
            traj_msg.data[i*6] = h_xu[i*(STATE_SIZE + CONTROL_SIZE)];
            traj_msg.data[i*6 + 1] = h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 1];
            traj_msg.data[i*6 + 2] = h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 2];
            traj_msg.data[i*6 + 3] = h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 3];
            traj_msg.data[i*6 + 4] = h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 4];
            traj_msg.data[i*6 + 5] = h_xu[i*(STATE_SIZE + CONTROL_SIZE) + 5];
        }
        traj_msg.data.push_back(current_trajopt_time_);
        traj_publisher_->publish(traj_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_subscription_; 
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr traj_publisher_;

    const uint32_t traj_length_; //length of xu trajectory
    const uint32_t goal_traj_length_; //length of full goal trajectory
    uint32_t traj_offset_; //offset of current trajectory
    double current_time_; //current time (updated by stateCallback)
    double current_trajopt_time_; //current time in trajopt loop
    double last_time_; 
    float *d_goal_eePos_traj_;
    float *d_xu_, *d_eePos_traj_, *d_lambda_;
    void *d_dynmem_const_;
    pcg_config<float> config_;
    float rho_ = 1e-3;
    float rho_reset_ = 1e-3;

    std::tuple<std::vector<int>, std::vector<double>, double, uint32_t, bool, std::vector<bool>> sqp_stats; //return from sqpSolvePcg

    std::vector<float> current_state_;
    std::mutex state_mutex_;
    std::mutex traj_mutex_;
    std::atomic<bool> state_updated_;

    std::thread mpc_thread_;
    std::atomic<bool> stop_trajopt_;
};

int main()
{

    char eePos_traj_file_name[100] = "figure8_traj_eePos_meters.csv";
	std::vector<std::vector<float>> goal_eePos_traj_2d = readCSVToVecVec<float>(eePos_traj_file_name);
    auto node = std::make_shared<TrajoptNode>(goal_eePos_traj_2d);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}