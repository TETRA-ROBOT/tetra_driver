#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace tetra {

/**
 * @brief TETRA 로봇 휠 모터 구동 및 오도메트리/TF 발행을 담당하는 ROS2 노드
 */
class TetraWheelDriverNode : public rclcpp::Node {
public:
    explicit TetraWheelDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    virtual ~TetraWheelDriverNode();

private:
    // 파라미터 초기화
    void loadParameters();
    
    // 콜백 함수
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void updateOdometry();

    // ROS2 통신 인터페이스
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 물리 기구학 파라미터
    double wheel_radius_{0.1033};
    double wheel_distance_{0.4330};
    double tread_width_{0.04};
    std::string port_{"/dev/ttyUSB0"};
    int baud_rate_{115200};
    bool use_ekf_{true};

    // 로봇 위치 및 속도 상태
    double x_pos_{0.0};
    double y_pos_{0.0};
    double theta_{0.0};
    double linear_vel_{0.0};
    double angular_vel_{0.0};
};

} // namespace tetra
