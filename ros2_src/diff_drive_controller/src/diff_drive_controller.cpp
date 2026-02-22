#include <cstdio>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class TwistToWheelSpeed : public rclcpp::Node
{
public:
  TwistToWheelSpeed()
  : Node("twist_to_wheel_speed")
  {
    
    // Get parameters
    wheel_separation_ = .67;
    wheel_radius_ = .11;
    
    // Subscribe to cmd_vel_unstamped
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/diff_drive_controller/cmd_vel_unstamped",
      10,
      std::bind(&TwistToWheelSpeed::twistCallback, this, std::placeholders::_1));
    
    // Create publishers for each wheel joint state
    wheel_l1_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_wheel_l1", 10);
    wheel_l2_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_wheel_l2", 10);
    wheel_l3_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_wheel_l3", 10);
    wheel_r1_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_wheel_r1", 10);
    wheel_r2_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_wheel_r2", 10);
    wheel_r3_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_wheel_r3", 10);
    
    RCLCPP_INFO(this->get_logger(), "Twist to Wheel Speed node started");
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %.3f m, Wheel radius: %.3f m", 
                wheel_separation_, wheel_radius_);
  }

private:
  void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Extract linear and angular velocities
    double linear_vel = msg->linear.x;   // m/s
    double angular_vel = msg->angular.z; // rad/s
    
    // Calculate wheel velocities using differential drive kinematics
    // v_left = (2*v - w*L) / 2
    // v_right = (2*v + w*L) / 2
    // where v = linear velocity, w = angular velocity, L = wheel separation
    
    double left_vel = linear_vel - (-angular_vel * wheel_separation_ / 2.0);
    double right_vel = linear_vel + (-angular_vel * wheel_separation_ / 2.0);
    
    // Convert linear velocities to angular velocities (rad/s)
    // omega = v / r
    double left_wheel_speed = left_vel / wheel_radius_;
    double right_wheel_speed = right_vel / wheel_radius_;
    
    // Create JointState messages for each wheel
    auto create_joint_state = [this](const std::string& name, double velocity) {
      sensor_msgs::msg::JointState joint_state;
      joint_state.header.stamp = this->now();
      joint_state.name = {name};
      joint_state.velocity = {velocity};
      return joint_state;
    };
    
    // Publish individual joint states
    wheel_l1_pub_->publish(create_joint_state("joint_wheel_l1", left_wheel_speed));
    wheel_l2_pub_->publish(create_joint_state("joint_wheel_l2", left_wheel_speed));
    wheel_l3_pub_->publish(create_joint_state("joint_wheel_l3", left_wheel_speed));
    wheel_r1_pub_->publish(create_joint_state("joint_wheel_r1", -right_wheel_speed));
    wheel_r2_pub_->publish(create_joint_state("joint_wheel_r2", -right_wheel_speed));
    wheel_r3_pub_->publish(create_joint_state("joint_wheel_r3", -right_wheel_speed));
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_l1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_l2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_l3_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_r1_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_r2_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_r3_pub_;
  
  double wheel_separation_;
  double wheel_radius_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistToWheelSpeed>());
  rclcpp::shutdown();
  return 0;
}
