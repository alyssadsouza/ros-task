#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("pd_controller")
  {
    // Create the subscriber
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/limobot2/limobot2/pose", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)
    );
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    std::cout << "Launched pd_controller" << std::endl;
  }

private:
  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Print the received message to the console
    std::cout << "Received: " << msg->pose.position.x << std::endl;
    geometry_msgs::msg::Twist drive_cmd;
    drive_cmd.linear.x = 0.25;
    publisher_->publish(drive_cmd);
    std::cout << "Drive: " << drive_cmd.linear.x << std::endl; 
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);  // Initialize ROS 2
  rclcpp::spin(std::make_shared<MinimalSubscriber>());  // Spin the node
  rclcpp::shutdown();  // Shutdown ROS 2
  return 0;
}
