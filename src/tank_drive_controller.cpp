#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "tank_interface/msg/motor_rpm_array.hpp"

class TankDriveController : public rclcpp::Node {
public:
    TankDriveController() : Node("tank_drive_controller") {
        this->declare_parameter("front_wheel_diameter", 0.186);
        this->declare_parameter("rear_wheel_diameter", 0.148);
        this->declare_parameter("wheel_separation", 0.515);
        this->declare_parameter("wheel_base", 0.700);

        front_wheel_diameter_ = this->get_parameter("front_wheel_diameter").as_double();
        rear_wheel_diameter_ = this->get_parameter("rear_wheel_diameter").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        wheel_base_ = this->get_parameter("wheel_base").as_double();

        twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&TankDriveController::twistCallback, this, std::placeholders::_1));

        motor_rpm_pub_ = this->create_publisher<tank_interface::msg::MotorRPMArray>("motor_rpms", 10);

        RCLCPP_INFO(this->get_logger(), "TankDriveController node started");
    }

private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        float front_left_rpm = calculateWheelRPM(msg->linear.x, msg->angular.z, front_wheel_diameter_, -wheel_separation_ / 2.0);
        float front_right_rpm = calculateWheelRPM(msg->linear.x, msg->angular.z, front_wheel_diameter_, wheel_separation_ / 2.0);
        float rear_left_rpm = calculateWheelRPM(msg->linear.x, msg->angular.z, rear_wheel_diameter_, -wheel_separation_ / 2.0);
        float rear_right_rpm = calculateWheelRPM(msg->linear.x, msg->angular.z, rear_wheel_diameter_, wheel_separation_ / 2.0);

        publishWheelRPM(front_left_rpm, front_right_rpm, rear_left_rpm, rear_right_rpm);
    }

    float calculateWheelRPM(float linear_velocity, float angular_velocity, float wheel_diameter, float wheel_offset) {
        float wheel_circumference = M_PI * wheel_diameter;
        return (linear_velocity + (angular_velocity * wheel_offset)) / wheel_circumference * 60.0;
    }

    void publishWheelRPM(float front_left, float front_right, float rear_left, float rear_right) {
        auto msg = tank_interface::msg::MotorRPMArray();
        msg.rpm = {front_left, front_right, rear_left, rear_right};
        motor_rpm_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published RPMs: [%.2f, %.2f, %.2f, %.2f]",
                    msg.rpm[0], msg.rpm[1], msg.rpm[2], msg.rpm[3]);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Publisher<tank_interface::msg::MotorRPMArray>::SharedPtr motor_rpm_pub_;

    float front_wheel_diameter_;
    float rear_wheel_diameter_;
    float wheel_separation_;
    float wheel_base_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TankDriveController>());
    rclcpp::shutdown();
    return 0;
}
