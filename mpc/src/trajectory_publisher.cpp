#include <chrono>
#include <functional>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class TrajectoryPublisher : public rclcpp::Node
{
public:
  TrajectoryPublisher()
  : Node("trajectory_publisher")
  {

    state_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "curr_state_and_time", 1, std::bind(&TrajectoryPublisher::stateCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_pos_traj", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5000), std::bind(&TrajectoryPublisher::publish_trajectory, this));
  }

private:
    void stateCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        current_state_.assign(msg->data.begin(), msg->data.begin() + 6);
        current_time_ = msg->data.back();
        RCLCPP_INFO(this->get_logger(), "Received state: %f, %f, %f, %f, %f, %f at time %f", current_state_[0], current_state_[1], current_state_[2], current_state_[3], current_state_[4], current_state_[5], current_time_);
    }

    void publish_trajectory()
    {
    auto message = std_msgs::msg::Float64MultiArray();

    // Create a simple trajectory
    // 6 joints, 32 knot points
    for (int i = 0; i < 32; ++i) {
        message.data.push_back(sin(i * 0.2) * 0.5);
        message.data.push_back(cos(i * 0.2) * 0.5);
        message.data.push_back(sin(i * 0.1) * 0.3);
        message.data.push_back(cos(i * 0.1) * 0.3);
        message.data.push_back(sin(i * 0.3) * 0.2);
        message.data.push_back(cos(i * 0.3) * 0.2);
    }
    double current_time = this->now().seconds();
    message.data.push_back(current_time);

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Published trajectory: [%f, %f, %f, %f, %f, %f] at time %f",
        message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5], current_time);
    }

    std::vector<float> current_state_;
    double current_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_subscription_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}