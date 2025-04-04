#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <iostream>
#include <math.h>
#include <cmath>

using namespace std::chrono_literals;


double normalizeAngle(double angle) {
  auto fac = fmod(angle, 2.0*M_PI);
  if (fac < 0) return fac + 2.0*M_PI;
  return fac;
}

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("pd_controller")
  {
    // Create the subscriber
    // subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //   "/limobot2/limobot2/pose", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1)
    // );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    std::cout << "Launched pd_controller" << std::endl;

    // Call on_timer function every second
    timer_ = this->create_wall_timer(0.01s, std::bind(&MinimalSubscriber::on_timer, this));

    initial_rotation_gain = 0.5;
    route_rotation_gain = 0.05;
    translation_gain = 0.1;
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

  void on_timer() {
    
    geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform("goal", "limobot2/base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));

    geometry_msgs::msg::Twist msg;

    tf2::Quaternion yawQuat;
    tf2::fromMsg(t.transform.rotation,yawQuat);
  
    auto current_yaw = normalizeAngle(tf2::getYaw(yawQuat));
    auto desired_yaw = normalizeAngle(atan(t.transform.translation.y/
      t.transform.translation.x));
    auto linear_error = std::hypot(t.transform.translation.x, t.transform.translation.y);
    double angle_diff = (desired_yaw - current_yaw);
    const double angle_threshold = 0.06; // 5 degrees in radians

    std::cout << t.transform.translation.x << ", " << t.transform.translation.y << ", " << angle_diff << ", " << linear_error << std::endl;

    if (fabs(angle_diff) > angle_threshold && !has_oriented) {
      msg.angular.z = 0.5*angle_diff;
    } else {
      has_oriented = true;
      msg.angular.z = 0.05*angle_diff;
      msg.linear.x = 0.5*linear_error;
    }

    publisher_->publish(msg);
    

  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
  double initial_rotation_gain;
  double route_rotation_gain;
  double translation_gain;
  bool has_oriented = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);  // Initialize ROS 2
  rclcpp::spin(std::make_shared<MinimalSubscriber>());  // Spin the node
  rclcpp::shutdown();  // Shutdown ROS 2
  return 0;
}
