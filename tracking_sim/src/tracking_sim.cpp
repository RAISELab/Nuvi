#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <vector>
#include <fstream>
#include <cmath>
#include "ament_index_cpp/get_package_share_directory.hpp"


using std::placeholders::_1;

class Tracking_sim : public rclcpp::Node
{
public:
  Tracking_sim() : Node("tracking_sim"), total_distance_(0.0), x3_reached_(false), x6_reached_(false)
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Tracking_sim::odom_callback, this, _1));
    start_time_ = this->now();
    prev_time_ = start_time_;
    RCLCPP_INFO(this->get_logger(), "tracking_sim started.");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto now = this->now();
    double dt = (now - prev_time_).seconds();
    prev_time_ = now;

    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double v = msg->twist.twist.linear.x;
    double elapsed_time = (now - start_time_).seconds();

    double velocity_at_x3_;


    // 거리 계산
    if (has_prev_pos_)
    {
      double dx = x - prev_x_;
      double dy = y - prev_y_;
      total_distance_ += std::sqrt(dx * dx + dy * dy);
    }
    prev_x_ = x;
    prev_y_ = y;
    has_prev_pos_ = true;

    // RCLCPP_INFO(this->get_logger(), "시간: %.2f s, 위치: (%.2f, %.2f), 속도: %.2f m/s, 누적 거리: %.2f m",
    //             elapsed_time, x, y, v, total_distance_);

    // 데이터 저장
    time_vec_.push_back(elapsed_time);
    velocity_vec_.push_back(v);
    distance_vec_.push_back(total_distance_);
    x_vec_.push_back(x);

    // x=3 도달
    if (!x3_reached_ && x >= 3.0)
    {
      x3_reached_ = true;
      time_at_x3_ = elapsed_time;
      distance_at_x3_ = total_distance_;
      velocity_at_x3_ = v;  
      // RCLCPP_INFO(this->get_logger(), "x=3 시간: %.2f s, 속도: %.2f m/s, 거리: %.2f m", elapsed_time, v, total_distance_);
    }

    // x=6 도달
    if (x3_reached_ && !x6_reached_ && x >= 6.0)
    {
      x6_reached_ = true;
      double time_at_x6 = elapsed_time;
      double distance_at_x6 = total_distance_;

      double avg1 = distance_at_x3_ / time_at_x3_;
      double avg2 = (distance_at_x6 - distance_at_x3_) / (time_at_x6 - time_at_x3_);
      double avg_total = total_distance_ / elapsed_time;

      RCLCPP_INFO(this->get_logger(), " x=6 도달! 시간: %.2f s, 거리: %.2f m", time_at_x6, distance_at_x6);
      RCLCPP_INFO(this->get_logger(), " 평균 속도");
      RCLCPP_INFO(this->get_logger(), " x=3까지: %.2f m/s", avg1);
      RCLCPP_INFO(this->get_logger(), " x=3 ~ x=6: %.2f m/s", avg2);
      RCLCPP_INFO(this->get_logger(), " 전체: %.2f m/s", avg_total);
      RCLCPP_INFO(this->get_logger(), " x=3 시간: %.2f s, 속도: %.2f m/s, 거리: %.2f m", time_at_x3_, velocity_at_x3_, distance_at_x3_);
      
      std::string package_path = ament_index_cpp::get_package_share_directory("tracking_sim");
      std::string save_dir = package_path + "/data";  // data 폴더 내부

      std::string summary_path = save_dir + "/result_summary.txt";

      // result_summary.txt 저장
      std::ofstream summary(summary_path);
      summary << "x=3 도달 시점 정보:\n";
      summary << " - 시간: " << time_at_x3_ << " s\n";
      summary << " - 속도: " << velocity_at_x3_ << " m/s\n";
      summary << " - 거리: " << distance_at_x3_ << " m\n\n";

      summary << "x=6 도달 시점:\n";
      summary << " - 시간: " << time_at_x6 << " s\n";
      summary << " - 거리: " << distance_at_x6 << " m\n\n";

      summary << "평균 속도:\n";
      summary << " - x=3까지: " << avg1 << " m/s\n";
      summary << " - x=3~6: " << avg2 << " m/s\n";
      summary << " - 전체: " << avg_total << " m/s\n";
      summary.close();

      RCLCPP_INFO(this->get_logger(), "요약 저장 완료: result_summary.txt");

      save_csv();
    }
  }

  void save_csv()
  {
    std::string package_path = ament_index_cpp::get_package_share_directory("tracking_sim");
    std::string save_dir = package_path + "/data";  // data 폴더 내부
    std::string csv_path = save_dir + "/odom_data.csv";

    std::ofstream file(csv_path);
    file << "Time,X,Velocity,Distance\n";
    for (size_t i = 0; i < time_vec_.size(); ++i)
    {
      file << time_vec_[i] << "," << x_vec_[i] << "," << velocity_vec_[i] << "," << distance_vec_[i] << "\n";
    }
    file.close();
    RCLCPP_INFO(this->get_logger(), "CSV 저장 완료: odom_data.csv");
  }

  // Subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;

  // 상태
  rclcpp::Time start_time_;
  rclcpp::Time prev_time_;
  bool has_prev_pos_ = false;

  // 위치 및 거리 추적
  double prev_x_;
  double prev_y_;
  double total_distance_;

  // 체크포인트
  bool x3_reached_;
  bool x6_reached_;
  double time_at_x3_;
  double distance_at_x3_;

  // 저장 데이터
  std::vector<double> time_vec_;
  std::vector<double> velocity_vec_;
  std::vector<double> distance_vec_;
  std::vector<double> x_vec_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tracking_sim>());
  rclcpp::shutdown();
  return 0;
}
