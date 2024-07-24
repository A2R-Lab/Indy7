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
        state_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("curr_state_and_time", 10);
        traj_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "joint_pos_traj", 1, std::bind(&RobotDriver::trajectory_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer( // frequency of robot control
            std::chrono::milliseconds(10), std::bind(&RobotDriver::timer_callback, this));

        //TODO: initialize indySDK
        //indy_ = std::make_unique<IndyDCP3>(robot_ip);
        //indy_->start_teleop(10);
    }

private:

    void trajectory_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        trajectory_.clear();
        for (size_t i = 0; i + 5 < msg->data.size(); i += 6) {
            std::vector<double> point(msg->data.begin() + i, msg->data.begin() + i + 6);
            trajectory_.push_back(point);
        }
        
        trajectory_start_time_ = msg->data.back();
        RCLCPP_INFO(this->get_logger(), "Received trajectory, trajectory_start_time: %f", trajectory_start_time_);
    }

    void timer_callback()
    {
        auto state_msg = std_msgs::msg::Float64MultiArray();
        
        //TODO: get control data from indy7 or sim instead
        state_msg.data = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};

        double current_time = this->now().seconds();
        state_msg.data.push_back(current_time);
        state_publisher_->publish(std::move(state_msg));

        // RCLCPP_INFO(this->get_logger(), "Current state and time: [%f, %f, %f, %f, %f, %f, %f]",
        //     state_msg.data[0], state_msg.data[1], state_msg.data[2],
        //     state_msg.data[3], state_msg.data[4], state_msg.data[5],
        //     state_msg.data[6]);

        std::lock_guard<std::mutex> lock(trajectory_mutex_);
        if (!trajectory_.empty()) {
            double elapsed_time = this->now().seconds() - trajectory_start_time_;
            int index = static_cast<int>(elapsed_time / timestep_);
            if (index < knot_points_ && index < static_cast<int>(trajectory_.size())) {
                //TODO: move to trajectory point
                std::cout << "Moving to trajectory point " << index;
                std::cout << "  [";
                for (size_t i = 0; i < trajectory_[index].size(); ++i) {
                    std::cout << trajectory_[index][i] << " ";
                }
                std::cout << "]" << std::endl;
            } else {
                RCLCPP_INFO(this->get_logger(), "Trajectory completed");
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
    auto node = std::make_shared<RobotDriver>(32, 0.125); //TODO: edit timestep, pass robot_ip
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}