/*********************************************************************
 * @file robot_driver_jpos.cpp
 
 * @brief RobotDriver class implementation.
 *********************************************************************/

#include <memory>
#include <vector>
#include <chrono>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "mpc_settings.hpp"
//TODO: import indySDK

/**
 * @brief RobotDriver class.
 * 
 * Driver for controlling Neuromeka Indy7 robot. Subscribes to the joint position trajectory, publishes the current state of the robot.
 *
 * @param knot_points Number of knot points in the trajectory
 * @param timestep Time step between knot points
 */
class RobotDriver : public rclcpp::Node
{
public:
    RobotDriver(int knot_points, double timestep)
    : Node("robot_driver"), knot_points_(knot_points), timestep_(timestep)
    {
        traj_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_pos_traj", 1, std::bind(&RobotDriver::trajectoryCallback, this, std::placeholders::_1));
        
        state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("curr_state_and_time", 10);


        RCLCPP_INFO(this->get_logger(), "Starting robot control and state publishing at %f hz", 1000.0 / ROBOT_CONTROL_PERIOD_MS);
        
        //timer for robot control and state publishing
        timer_ = this->create_wall_timer(std::chrono::milliseconds(ROBOT_CONTROL_PERIOD_MS), std::bind(&RobotDriver::timerCallback, this));

        //TODO: initialize indySDK
        //indy_ = std::make_unique<IndyDCP3>(robot_ip);
        //indy_->start_teleop(10);
    }

private:
    /**
     * @brief Callback function for the joint position trajectory, updates instance variable trajectory_ whenever a new trajectory is received.
     * 
     * @param msg Joint position trajectory message
     */
    void trajectoryCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_.clear();
        //msg->data is state_size * knot_points + 1 = = 6 * 32 + 1 = 193
        for (size_t i = 0; i + 5 < msg->data.size(); i += 6) { 
            std::vector<double> point(msg->data.begin() + i, msg->data.begin() + i + 6);
            trajectory_.push_back(point);
        }
        trajectory_start_time_ = msg->data.back();
    
        RCLCPP_INFO(this->get_logger(), "<trajectory_callback()>: Received trajectory. Knot points = %d, t_0 = %f", static_cast<int>(trajectory_.size()), trajectory_start_time_);
    }

    /**
     * @brief Timer callback function for robot control and state publishing.
     * Robot is commanded to move to certain points in the trajectory based on the elapsed time.
     */
    void timerCallback()
    {
        auto state_msg = std_msgs::msg::Float64MultiArray();
        
        //TODO: get control data from indy7 or sim instead

        //if empty or nan
        if (trajectory_.empty() || std::isnan(trajectory_[0][0]))
        {
            state_msg.data = {0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001};       
        }
        else
        {
            state_msg.data = {trajectory_[1][0], trajectory_[1][1], trajectory_[1][2], trajectory_[1][3], trajectory_[1][4], trajectory_[1][5]};
        }

 
        

        double current_time = this->now().seconds();
        state_msg.data.push_back(current_time);
        state_publisher_->publish(std::move(state_msg)); //publish current state and time

        // Move to trajectory point based on elapsed time
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (!trajectory_.empty()) {
            double elapsed_time = current_time - trajectory_start_time_;
            int index = static_cast<int>(elapsed_time / timestep_);
            if (index < knot_points_ && index < static_cast<int>(trajectory_.size())) {
                RCLCPP_INFO(this->get_logger(), "<timerCallback()>: Moving to point %d: [%f, %f, %f, %f, %f, %f], elapsed time = %f, current time = %f",
                    index, trajectory_[index][0], trajectory_[index][1], trajectory_[index][2], trajectory_[index][3], trajectory_[index][4], trajectory_[index][5], elapsed_time, current_time);
                //TODO: move to trajectory point
            } else {
                //TODO: move to last point
                RCLCPP_INFO(this->get_logger(), "<timerCallback()>: Trajectory completed");
                trajectory_.clear();
            }
        }
        //TODO: anything else required in indySDK
    }

    int knot_points_;
    double timestep_;
    bool use_sim_time_;
    double trajectory_start_time_;
    std::vector<std::vector<double>> trajectory_;
    std::mutex trajectory_mutex_;
    //std::unique_ptr<IndyDCP3> indy_; //TODO: replace with indySDK

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr traj_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>(KNOT_POINTS, TIMESTEP); //TODO: edit timestep, pass robot_ip
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}