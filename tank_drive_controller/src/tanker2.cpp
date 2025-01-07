#include "tank_drive_controller/advanced_tank_drive_controller.hpp"

namespace tank_controller {

AdvancedTankController::AdvancedTankController() 
    : Node("advanced_tank_controller")
    , x_(0.0)
    , y_(0.0)
    , theta_(0.0) {
    
    initializeParameters();
    setupSubscribersAndPublishers();
    setupTimer();

    last_cmd_time_ = this->now();
    last_odom_time_ = this->now();
    wheel_diameter_ratio_ = front_wheel_diameter_ / rear_wheel_diameter_;

    RCLCPP_INFO(this->get_logger(), "Advanced Tank Controller initialized");
    RCLCPP_INFO(this->get_logger(), "Front wheel diameter: %.3f m", front_wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "Rear wheel diameter: %.3f m", rear_wheel_diameter_);
    RCLCPP_INFO(this->get_logger(), "Wheel separation: %.3f m", wheel_separation_);
    RCLCPP_INFO(this->get_logger(), "Max linear velocity: %.2f m/s", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "Max angular velocity: %.2f rad/s", max_angular_velocity_);
}

void AdvancedTankController::initializeParameters() {
    // Physical parameters (in meters)
    this->declare_parameter("front_wheel_diameter", 0.186);
    this->declare_parameter("rear_wheel_diameter", 0.148);
    this->declare_parameter("wheel_separation", 0.515);
    this->declare_parameter("wheel_base", 0.700);

    // Command limits
    this->declare_parameter("max_linear_velocity", 1.0);      // m/s
    this->declare_parameter("max_angular_velocity", 2.0);     // rad/s
    this->declare_parameter("max_linear_acceleration", 0.5);  // m/s^2
    this->declare_parameter("max_angular_acceleration", 1.0); // rad/s^2
    this->declare_parameter("max_rpm", 100.0);               // RPM

    // Load parameters
    front_wheel_diameter_ = this->get_parameter("front_wheel_diameter").as_double();
    rear_wheel_diameter_ = this->get_parameter("rear_wheel_diameter").as_double();
    wheel_separation_ = this->get_parameter("wheel_separation").as_double();
    wheel_base_ = this->get_parameter("wheel_base").as_double();

    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    max_linear_acceleration_ = this->get_parameter("max_linear_acceleration").as_double();
    max_angular_acceleration_ = this->get_parameter("max_angular_acceleration").as_double();
    max_rpm_ = this->get_parameter("max_rpm").as_double();
}

void AdvancedTankController::setupSubscribersAndPublishers() {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, 
        std::bind(&AdvancedTankController::cmdVelCallback, this, std::placeholders::_1));

    motor_rpm_pub_ = this->create_publisher<tank_interface::msg::MotorRPMArray>("motor_rpms", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void AdvancedTankController::setupTimer() {
    // Update timer (50Hz)
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&AdvancedTankController::updateCommand, this));
}

void AdvancedTankController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Limit incoming velocities
    target_twist_.linear.x = std::clamp(msg->linear.x, 
                                      -max_linear_velocity_, 
                                      max_linear_velocity_);
    target_twist_.angular.z = std::clamp(msg->angular.z, 
                                       -max_angular_velocity_, 
                                       max_angular_velocity_);
    last_cmd_time_ = this->now();
}

void AdvancedTankController::updateCommand() {
    // Check command timeout (500ms)
    if ((this->now() - last_cmd_time_).seconds() > 0.5) {
        target_twist_ = geometry_msgs::msg::Twist();
    }

    // Calculate time delta
    static auto last_update = this->now();
    double dt = (this->now() - last_update).seconds();
    last_update = this->now();

    // Apply acceleration limits
    updateVelocityWithLimits(current_twist_.linear.x, target_twist_.linear.x,
                            max_linear_acceleration_, dt);
    updateVelocityWithLimits(current_twist_.angular.z, target_twist_.angular.z,
                            max_angular_acceleration_, dt);

    // Calculate and publish wheel RPMs
    auto wheel_rpms = calculateWheelRPMs(current_twist_);
    limitAndPublishRPMs(wheel_rpms);
}

void AdvancedTankController::updateVelocityWithLimits(double& current, double target,
                                                     double max_acceleration, double dt) {
    double velocity_error = target - current;
    double max_velocity_change = max_acceleration * dt;
    
    if (std::abs(velocity_error) > max_velocity_change) {
        current += max_velocity_change * (velocity_error > 0 ? 1.0 : -1.0);
    } else {
        current = target;
    }
}

double AdvancedTankController::velocityToRPM(double velocity, double wheel_diameter) const {
    // Convert linear velocity (m/s) to RPM
    // RPM = (velocity * 60) / (π * diameter)
    return (velocity * 60.0) / (M_PI * wheel_diameter);
}

WheelState AdvancedTankController::calculateWheelRPMs(const geometry_msgs::msg::Twist& twist) const {
    WheelState result;
    
    // Calculate track velocities (m/s)
    double left_velocity = twist.linear.x - (twist.angular.z * wheel_separation_ / 2.0);
    double right_velocity = twist.linear.x + (twist.angular.z * wheel_separation_ / 2.0);

    // Convert velocities to RPM for rear wheels
    result.rear_left_rpm = velocityToRPM(left_velocity, rear_wheel_diameter_);
    result.rear_right_rpm = velocityToRPM(right_velocity, rear_wheel_diameter_);

    // Calculate front wheel RPMs
    result.front_left_rpm = velocityToRPM(left_velocity, front_wheel_diameter_);
    result.front_right_rpm = velocityToRPM(right_velocity, front_wheel_diameter_);

    return result;
}

void AdvancedTankController::limitAndPublishRPMs(const WheelState& wheel_rpms) {
    auto msg = tank_interface::msg::MotorRPMArray();
    
    // Apply RPM limits and convert to float
    msg.rpm = {
        static_cast<float>(std::clamp(wheel_rpms.front_left_rpm, -max_rpm_, max_rpm_)),
        static_cast<float>(std::clamp(wheel_rpms.front_right_rpm, -max_rpm_, max_rpm_)),
        static_cast<float>(std::clamp(wheel_rpms.rear_left_rpm, -max_rpm_, max_rpm_)),
        static_cast<float>(std::clamp(wheel_rpms.rear_right_rpm, -max_rpm_, max_rpm_))
    };
    
    motor_rpm_pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), 
                "RPMs (FL, FR, RL, RR): %.1f, %.1f, %.1f, %.1f",
                msg.rpm[0], msg.rpm[1], msg.rpm[2], msg.rpm[3]);
}

} // namespace tank_controller

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<tank_controller::AdvancedTankController>());
    rclcpp::shutdown();
    return 0;
}