#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "libroboclaw/roboclaw_driver.h"

const int ROBOCLAW_MAX_SPEED = 127;

class RoboClawJoy : public rclcpp::Node {
private:
    // Parameters
    std::string port_;
    int baudrate_;
    int address_;
    int axis_left_;
    int axis_right_;
    double publish_hz_;
    std::string roboclaw_left_encoder_topic_;
    std::string roboclaw_right_encoder_topic_;

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_encoder_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_encoder_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // RoboClaw driver
    libroboclaw::driver* roboclaw_;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Get joystick values [-1.0, 1.0]
        float left_pctg = (msg->axes.size() > 2) ? msg->axes[axis_left_] : 0.0f;
        float right_pctg = (msg->axes.size() > 2) ? msg->axes[axis_right_] : 0.0f;

        // Convert to output range: [-127, 127]
        int left_speed = static_cast<int>(left_pctg * ROBOCLAW_MAX_SPEED);
        int right_speed = static_cast<int>(right_pctg * ROBOCLAW_MAX_SPEED);

        RCLCPP_INFO(this->get_logger(), "left: %f%% Right: %f%% -> Left: %d%% Right: %d%%",
                    left_pctg, right_pctg, left_speed, right_speed);

        //drive_motors(left_speed, right_speed);
    }

    void drive_motors(int left_speed, int right_speed) {
        RCLCPP_INFO(this->get_logger(), "Driving Motors with the following speeds: Left: %d%% Right: %d%%",
                    left_speed, right_speed);

        roboclaw_->set_velocity(address_, std::make_pair(left_speed, right_speed));
    }

    void timer_callback() {
        // Publish encoder data
        auto encoders = roboclaw_->get_encoders(address_);

        auto left_msg = std_msgs::msg::Float32();
        left_msg.data = static_cast<float>(encoders.first);
        left_encoder_pub_->publish(left_msg);

        auto right_msg = std_msgs::msg::Float32();
        right_msg.data = static_cast<float>(encoders.second);
        right_encoder_pub_->publish(right_msg);
    }

public:
    RoboClawJoy() : Node("simple_roboclaw") {
        // Declare parameters
        this->declare_parameter<std::string>("port", "/dev/ttyACM0");
        this->declare_parameter<int>("baudrate", 38400);
        this->declare_parameter<int>("address", 128);
        this->declare_parameter<int>("axis_left", 1);  // Left stick vertical
        this->declare_parameter<int>("axis_right", 3); // Left stick horizontal
        this->declare_parameter<double>("publish_hz", 20.0);
        this->declare_parameter<std::string>("roboclaw_left_encoder_topic", "/roboclaw/left_encoder_data");
        this->declare_parameter<std::string>("roboclaw_right_encoder_topic", "/roboclaw/right_encoder_data");

        // Get parameters
        port_ = this->get_parameter("port").as_string();
        baudrate_ = this->get_parameter("baudrate").as_int();
        address_ = this->get_parameter("address").as_int();
        axis_left_ = this->get_parameter("axis_left").as_int();
        axis_right_ = this->get_parameter("axis_right").as_int();
        publish_hz_ = this->get_parameter("publish_hz").as_double();
        roboclaw_left_encoder_topic_ = this->get_parameter("roboclaw_left_encoder_topic").as_string();
        roboclaw_right_encoder_topic_ = this->get_parameter("roboclaw_right_encoder_topic").as_string();

        // Initialize RoboClaw driver
        /*
        try {
            roboclaw_ = new libroboclaw::driver(port_, baudrate_);
            RCLCPP_INFO(this->get_logger(), "Connected to RoboClaw on %s", port_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to RoboClaw: %s", e.what());
            throw;
        }

        try {
            roboclaw_->reset_encoders(address_);
            RCLCPP_INFO(this->get_logger(), "Successfully reset encoders");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to reset encoders: %s", e.what());
        }*/

        // Subscribe to joystick
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&RoboClawJoy::joy_callback, this, std::placeholders::_1));

        // Publishers
        left_encoder_pub_ = this->create_publisher<std_msgs::msg::Float32>(roboclaw_left_encoder_topic_, 10);
        right_encoder_pub_ = this->create_publisher<std_msgs::msg::Float32>(roboclaw_right_encoder_topic_, 10);

        // Timer
        auto period = std::chrono::duration<double>(1.0 / std::max(publish_hz_, 1e-3));
        timer_ = this->create_wall_timer(period, std::bind(&RoboClawJoy::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Simple RoboClaw controller ready!");
    }

    ~RoboClawJoy() {
        // Stop motors on shutdown
        /*
        try {
            roboclaw_->set_velocity(address_, std::make_pair(0, 0));  // Stop both motors
            roboclaw_->reset_encoders(address_);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "System had issues shutting down: %s", e.what());
        }*/
        delete roboclaw_;
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoboClawJoy>());
    rclcpp::shutdown();
    return 0;
}