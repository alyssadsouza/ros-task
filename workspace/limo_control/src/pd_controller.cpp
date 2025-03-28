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
  bool first_callback = false;
  double offset = 2.53;
  double k_p = 1;
  double original_pose_y = 0;
  double goal_y = 0;
  double prev_err = 0;

  double pose_y = 0;
  double del_t = 0.01;

  double calculate_vel() {
    // v_x = k_p * e_x(t) + k_d * de_x(t)/dt
    double err = goal_y - pose_y;
    double vel = k_p * err;
    prev_err = err;
    std::cout << err << std::endl;
    return vel;
  }

  void topic_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double msg_pose_y = msg->pose.position.y; 
    geometry_msgs::msg::Twist drive_cmd;
    if (!first_callback) {
      original_pose_y = msg_pose_y;
      goal_y = original_pose_y + offset;
      first_callback = true;
    }
    // Print the received message to the console
    std::cout << "Received: " << msg->pose.position.x << ", " << msg->pose.position.y << std::endl;
    pose_y = msg_pose_y;
    drive_cmd.linear.x = calculate_vel();
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
