#ifndef TANK_DRIVE_CONTROLLER_ADVANCED_TANK_DRIVE_CONTROLLER_HPP_
#define TANK_DRIVE_CONTROLLER_ADVANCED_TANK_DRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tank_interface/msg/motor_rpm_array.hpp"

namespace tank_controller {

struct WheelState {
    float front_left_rpm;
    float front_right_rpm;
    float rear_left_rpm;
    float rear_right_rpm;
};

class AdvancedTankController : public rclcpp::Node {
public:
    explicit AdvancedTankController();
    virtual ~AdvancedTankController() = default;

private:
    // ROS2 communication handles
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<tank_interface::msg::MotorRPMArray>::SharedPtr wheel_rpm_sub_;
    rclcpp::Publisher<tank_interface::msg::MotorRPMArray>::SharedPtr motor_rpm_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;

    // Physical parameters (all in meters)
    double front_wheel_diameter_;    // meters
    double rear_wheel_diameter_;     // meters
    double wheel_separation_;        // meters
    double wheel_base_;             // meters
    double wheel_diameter_ratio_;    // unitless

    // Command limits
    double max_linear_velocity_;     // m/s
    double max_angular_velocity_;    // rad/s
    double max_linear_acceleration_; // m/s^2
    double max_angular_acceleration_;// rad/s^2
    double max_rpm_;                // RPM
    int odom_frequency_;
    int motor_command_frequency_;

    // Current state
    geometry_msgs::msg::Twist current_twist_;
    geometry_msgs::msg::Twist target_twist_;
    geometry_msgs::msg::Twist actual_twist_;
    rclcpp::Time last_cmd_time_;
    WheelState current_wheel_state_;
    
    // Odometry state
    double x_, y_, theta_;
    rclcpp::Time last_odom_time_;

    // Initialization functions
    void initializeParameters();
    void setupSubscribersAndPublishers();
    void setupTimers();

    // Callback functions
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void wheelRPMCallback(const tank_interface::msg::MotorRPMArray::SharedPtr msg);
    
    // Update functions
    void updateCommand();
    void updateOdometry();
    void updateVelocityWithLimits(double& current, double target, 
                                 double max_acceleration, double dt);

    // Publishing functions
    void publishOdometry();
    void publishTransform(const nav_msgs::msg::Odometry& odom_msg);

    // Utility functions
    WheelState calculateWheelRPMs(const geometry_msgs::msg::Twist& twist) const;
    double velocityToRPM(double velocity, double wheel_diameter) const;
    double rpmToVelocity(double rpm, double wheel_diameter) const;
    void limitAndPublishRPMs(const WheelState& wheel_rpms);
};

} // namespace tank_controller

#endif // TANK_DRIVE_CONTROLLER_ADVANCED_TANK_DRIVE_CONTROLLER_HPP_