// ROS 2 node: corridor/wall following for a mecanum robot via two-point
// (bang-bang) control over an array of 8 ToF distance sensors.
// Ported from the ROS 1 node (Aut4 study project, 2020).
#include <array>
#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

class MecanumNavigationNode : public rclcpp::Node
{
public:
  MecanumNavigationNode()
  : Node("mecanum_navigation_node")
  {
    sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "robot1/tof", 1,
      std::bind(&MecanumNavigationNode::tofCallback, this, std::placeholders::_1));

    pub_ = create_publisher<geometry_msgs::msg::Twist>("robot1/cmd_vel", 1);

    // Replaces the ROS 1 ros::Rate(100) + while(ros::ok()) loop: a 100 Hz timer.
    timer_ = create_wall_timer(10ms, std::bind(&MecanumNavigationNode::onTimer, this));
  }

private:
  void tofCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // Copy the ToF readings into the fixed buffer (guard against >8 values).
    const size_t n = std::min(msg->data.size(), tof_.size());
    for (size_t i = 0; i < n; ++i) {
      tof_[i] = msg->data[i];
    }
  }

  void onTimer()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = 1.5;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;

    // Two-point (bang-bang) control: balance the two lateral sensor pairs.
    const float e = (tof_[4] - tof_[5]) + (tof_[2] - tof_[3]);
    if (e < 0) {
      msg.angular.z = -0.7;
    }
    if (e > 0) {
      msg.angular.z = 0.7;
    }

    pub_->publish(msg);
  }

  std::array<float, 8> tof_{};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MecanumNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
