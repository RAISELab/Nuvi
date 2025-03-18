#include <cmath>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// tf2 변환 관련 헤더 (쿼터니언 -> RPY)
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class Test_tetra : public rclcpp::Node
{
public:
  Test_tetra()
  : Node("test_tetra"),
    rotating_(false),
    done_(false),
    current_yaw_(0.0),
    initial_yaw_(0.0)
  {
    // (1) 오도메트리 구독: 로봇의 현재 자세(orientation) 파악
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&Test_tetra::odomCallback, this, std::placeholders::_1));
    // (2) 속도 명령 퍼블리셔: 회전 명령을 로봇에 전달
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS());

    // (3) 주기적 제어 타이머: 10Hz(100ms)마다 controlLoop() 실행
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Test_tetra::controlLoop, this));

    // 목표 회전 각도
    target_angle_rad_ = 30.0 * M_PI / 180.0;
    RCLCPP_INFO(this->get_logger(), "Test_tetra initialized. Target angle: 30 deg");
  }

private:
  // (A) 오도메트리 콜백
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // 쿼터니언 -> (roll, pitch, yaw) 변환
    const auto &q = msg->pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 mat(quat);

    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    current_yaw_ = yaw;  // 로봇의 현재 헤딩(rad)
  }

  // (B) 주기적 제어 루프
  void controlLoop()
  {
    // 이미 회전 완료라면 아무 것도 하지 않음
    if (done_) {
      return;
    }

    // 아직 회전을 시작하지 않았다면, 회전 시작값(초기 yaw) 기록
    if (!rotating_) {
      rotating_ = true;
      initial_yaw_ = current_yaw_;
      RCLCPP_INFO(this->get_logger(), "Start rotating from %.2f rad", initial_yaw_);
    }

    // 현재 yaw와 초기 yaw의 차이(정규화)
    double diff = normalizeAngle(current_yaw_ - initial_yaw_);
    RCLCPP_INFO(this->get_logger(), "rotating %.2f rad", diff);    

    // 목표 각도에 도달했는지 확인
    if (std::fabs(diff) >= std::fabs(target_angle_rad_)) {
      // (1) 속도 0으로 정지
      geometry_msgs::msg::Twist stop_cmd;
      cmd_vel_pub_->publish(stop_cmd);

      // (2) 플래그 설정
      done_ = true;
      RCLCPP_INFO(this->get_logger(),
                  "Reached 30 deg. Final diff = %.2f rad (%.2f deg). Stopping.",
                  diff, diff * 180.0 / M_PI);
      return;
    }

    // 목표 각도에 도달하지 않았다면 계속 회전 명령
    geometry_msgs::msg::Twist cmd;
    double sign = (target_angle_rad_ > 0.0) ? 1.0 : -1.0;
    cmd.angular.z = 0.05 * sign;  // 회전 속도(0.2 rad/s) 예시
    cmd_vel_pub_->publish(cmd);
  }

  // (C) 보조 함수: -π ~ π 범위로 각도 정규화
  double normalizeAngle(double angle)
  {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

private:
  // ROS 관련 멤버
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 회전 제어용 멤버
  bool rotating_;
  bool done_;
  double current_yaw_;      // 현재 헤딩(rad)
  double initial_yaw_;      // 회전 시작 시점의 헤딩(rad)
  double target_angle_rad_; // 목표 회전 각도(rad)
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Test_tetra>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
