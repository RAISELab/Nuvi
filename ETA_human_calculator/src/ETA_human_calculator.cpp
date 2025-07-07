#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <set>
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>             
#include <tf2/LinearMath/Matrix3x3.h>                    
#include <geometry_msgs/msg/vector3.hpp>     
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int64.hpp>


using std::placeholders::_1;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class EtaOnceSubscriber : public rclcpp::Node {
public:
    EtaOnceSubscriber() : Node("eta_once_subscriber"), eta_received_(false), map_received_(false), people_loaded_(false) {

        congestion_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
            "/congestion_info", 10);
        
        // 방향 퍼블리셔
        direction_pub_ = this->create_publisher<std_msgs::msg::Int64>(
            "/direction", 10);

        // cmd_vel 구독
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&EtaOnceSubscriber::cmdVelCallback, this, _1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,  
            std::bind(&EtaOnceSubscriber::pathCallback, this, _1));

        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.transient_local();
        qos.reliable();
        rclcpp::QoS odom_qos(rclcpp::KeepLast(10));  // 일반적인 odom QoS
        odom_qos.best_effort();  // 또는 .reliable()도 가능 (퍼블리셔에 맞춰야 함)  

        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", odom_qos,
            std::bind(&EtaOnceSubscriber::odomCallback, this, _1));

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", qos,
            std::bind(&EtaOnceSubscriber::mapCallback, this, _1));

        
        starting_speed_ = {0.220, 0.219, 0.218, 0.217, 0.216, 0.215};

        velocity_change_ = {
            { 0.0,  0.001,  0.002,  0.003,  0.004,  0.005},
            {-0.001,  0.0,  0.001,  0.002,  0.003,  0.004},
            {-0.002, -0.001,  0.0,  0.001,  0.002,  0.003},
            {-0.003, -0.002, -0.001,  0.0,  0.001,  0.002},
            {-0.004, -0.003, -0.002, -0.001,  0.0,  0.001},
            {-0.005, -0.004, -0.003, -0.002, -0.001,  0.0}
        };

        distance_change_ = {
            {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.05, 0.0, 0.0, 0.0, 0.0, 0.0},
            {0.1, 0.05, 0.0, 0.0, 0.0, 0.0},
            {0.15, 0.1, 0.05, 0.0, 0.0, 0.0},
            {0.2, 0.15, 0.1, 0.05, 0.0, 0.0},
            {0.25, 0.2, 0.15, 0.1, 0.05, 0.0}
        };

        starting_distance_change_ = {0.0, 0.05, 0.10, 0.15, 0.20, 0.25};

        debug_out_.open("/home/min/grid_debug_log.csv", std::ios::out | std::ios::trunc);
        debug_out_ << "grid_idx,row,col,people,raw_distance,sigmoid_boosted_distance,velocity,next_velocity,time_add,total_eta,timestamp\n";

    }

    ~EtaOnceSubscriber() {
        if (debug_out_.is_open()) {
            debug_out_.close();
        }
    }

private:

    double sigmoid_boost(double x, double k = 1.0) {
        return 2.0 / (1.0 + std::exp(-k * (x - 3.0)));
    }

    double getYawFromPose(const geometry_msgs::msg::Pose& pose) {
        tf2::Quaternion q(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        return yaw;  // radian
    }

    double calculate_congestion(int people_count) {
        const double cylinder_radius = 0.15;
        const double inflation = 0.55;
        const double radius_total = cylinder_radius + inflation;
        const double cylinder_area = M_PI * radius_total * radius_total;

        // 3x3 그리드 영역의 실제 면적 (m²)
        const double grid_area = 9;
        const double scaling_factor = 100.0 / grid_area;

        return cylinder_area * people_count * scaling_factor;
    }       

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        int64_t direction = 0;
        std::string direction_str;

        if (msg->angular.z > 0.05) {
            direction = 1;
            direction_str = "왼쪽";
        } else if (msg->angular.z < -0.05) {
            direction = -1;
            direction_str = "오른쪽";
        } else {
            direction = 0;
            direction_str = "직진";
        }

        // 방향이 바뀐 경우에만 로그 및 퍼블리시
        if (direction != last_direction_) {
            last_direction_ = direction;

            // 로그 출력
            RCLCPP_INFO(this->get_logger(), "방향 판단: %s (angular.z = %.3f)", direction_str.c_str(), msg->angular.z);

            // 퍼블리시
            std_msgs::msg::Int64 dir_msg;
            dir_msg.data = direction;
            direction_pub_->publish(dir_msg);
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        pose_received_ = true;

        // RCLCPP_INFO(this->get_logger(), "[odom] x=%.2f y=%.2f yaw=%.2f",
        //     current_pose_.position.x,
        //     current_pose_.position.y,
        //     getYawFromPose(current_pose_));

        checkSurroundingCongestion();
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_info_ = msg->info;
        map_received_ = true;
        RCLCPP_INFO(this->get_logger(), "\u2705 map 수신 완료");
    }

    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        if (!people_loaded_) {
            readPeopleMatrix("/home/min/turtlebot3_ws/src/ETA_human_calculator/data/people_matrix.txt");
        }
        if (!map_received_ || !people_loaded_) {
            RCLCPP_WARN(this->get_logger(), "지도 또는 사람 수 정보가 준비되지 않음");
            return;
        }

        int grid_px = 60;
        int margin_px = 20;
        int usable_height = map_info_.height - 2 * margin_px;
        int usable_width = map_info_.width - 2 * margin_px;
        int grid_rows = usable_height / grid_px;
        int grid_cols = usable_width / grid_px;

        double cell_width = grid_px * map_info_.resolution;
        double cell_height = grid_px * map_info_.resolution;

        std::vector<std::vector<double>> grid_dist(grid_rows, std::vector<double>(grid_cols, 0.0));
        std::vector<int> grid_visit_order;
        std::set<int> visited;

        for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
            auto &p1 = msg->poses[i].pose.position;
            auto &p2 = msg->poses[i + 1].pose.position;

            double mid_x = (p1.x + p2.x) / 2.0;
            double mid_y = (p1.y + p2.y) / 2.0;
            double length = std::hypot(p2.x - p1.x, p2.y - p1.y);

            if (mid_x < 0 || mid_y < 0) continue;

            int col = std::clamp(static_cast<int>(mid_x / cell_width), 0, grid_cols - 1);
            int row = std::clamp(static_cast<int>(mid_y / cell_height), 0, grid_rows - 1);

            if (row < 0 || row >= grid_rows || col < 0 || col >= grid_cols) continue;

            grid_dist[row][col] += length;

            int index = row * grid_cols + col;
            if (visited.find(index) == visited.end()) {
                grid_visit_order.push_back(index);
                visited.insert(index);
            }
        }

        double time_estimate = 0.0;
        double current_speed = 0.0;

        if (!grid_visit_order.empty()) {

            std::time_t t = std::time(nullptr);
            char timestamp[100];
            std::strftime(timestamp, sizeof(timestamp), "%F %T", std::localtime(&t));
            int first_idx = grid_visit_order[0];
            int first_row = first_idx / grid_cols;
            int first_col = first_idx % grid_cols;

            int first_people = people_matrix_[first_row][first_col];
            double grid1_dis = grid_dist[first_row][first_col] + starting_distance_change_[first_people];
            current_speed = starting_speed_[first_people];
            time_estimate = grid1_dis / current_speed;

            debug_out_ << first_idx<< "," << first_row << "," << first_col << "," << first_people << ","
                       << grid_dist[first_row][first_col] << "," << grid1_dis << ","
                       << current_speed << "," << 0 << ","
                       << 0 << "," << time_estimate << ","
                       << timestamp << "\n";;            

            for (size_t i = 1; i < grid_visit_order.size(); ++i) {
                int curr_idx = grid_visit_order[i];
                int row = curr_idx / grid_cols;
                int col = curr_idx % grid_cols;

                int people = people_matrix_[row][col];
                double raw_dis_change = distance_change_[first_people][people];
                double boosted = sigmoid_boost(raw_dis_change);
                double grid_dis = grid_dist[row][col] + boosted;
                double next_speed = current_speed + velocity_change_[first_people][people];
                if (next_speed < 0.1) next_speed = 0.1;

                double time_add = grid_dis / next_speed;
                if (i == grid_visit_order.size() - 1) {
                    time_add *= 1.5;
                }               

                time_estimate += time_add;

                std::time_t t = std::time(nullptr);
                std::strftime(timestamp, sizeof(timestamp), "%F %T", std::localtime(&t));

                debug_out_ << curr_idx << "," << row << "," << col << "," << people << ","
                           << grid_dist[row][col] << "," << grid_dis << ","
                           << current_speed << "," << next_speed << ","
                           << time_add << "," << time_estimate << ","
                           << timestamp << "\n";;

                first_people = people;
                current_speed = next_speed;
            }

            // RCLCPP_INFO(this->get_logger(), "\u23F2 전체 보정 ETA: %.2f 초", time_estimate);
        }
    }


    bool readPeopleMatrix(const std::string &filename) {
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "people_matrix.txt 열 수 없음");
            return false;
        }

        int grid_px = 60;
        int margin_px = 20;
        int usable_height = map_info_.height - 2 * margin_px;
        int usable_width = map_info_.width - 2 * margin_px;
        int grid_rows = usable_height / grid_px;
        int grid_cols = usable_width / grid_px;

        people_matrix_.resize(grid_rows, std::vector<int>(grid_cols, 0));
        std::string line;
        int row = 0;

        while (std::getline(infile, line) && row < grid_rows) {
            std::istringstream iss(line);
            for (int col = 0; col < grid_cols; ++col) {
                int val;
                if (!(iss >> val)) {
                    RCLCPP_ERROR(this->get_logger(), "people_matrix.txt 형식 오류 (row %d)", row);
                    return false;
                }
                people_matrix_[row][col] = val;
            }
            row++;
        }

        if (row != grid_rows) {
            RCLCPP_ERROR(this->get_logger(), "people_matrix.txt는 %d행이어야 함", grid_rows);
            return false;
        }

        // RCLCPP_INFO(this->get_logger(), "\u2714 people_matrix.txt 읽기 완료");
        people_loaded_ = true;
        return true;
    }

    void checkSurroundingCongestion() {
        if (!pose_received_ || !map_received_ || !people_loaded_) {
            RCLCPP_WARN(this->get_logger(), "pose/map/people 정보가 준비되지 않음");
            return;
        }

        double yaw = getYawFromPose(current_pose_);
        int grid_px = 60;
        int margin_px = 20;
        int grid_rows = (map_info_.height - 2 * margin_px) / grid_px;
        int grid_cols = (map_info_.width - 2 * margin_px) / grid_px;
        double cell_size = grid_px * map_info_.resolution;

        // 현재 위치
        double x = current_pose_.position.x;
        double y = current_pose_.position.y;

        // 결과 변수 초기화 (-1: 범위 밖)
        int front_congestion = -1;
        int left_congestion = -1;
        int right_congestion = -1;

        std::vector<std::pair<std::string, double>> directions = {
            {"front", yaw},
            {"left", yaw + M_PI / 4},
            {"right", yaw - M_PI / 4}
        };

        for (auto& [label, dir_yaw] : directions) {
            // 방향 기준으로 1칸 앞 좌표 계산
            double check_x = x + cell_size * std::cos(dir_yaw);
            double check_y = y + cell_size * std::sin(dir_yaw);

            // 전체 맵에서 offset 고려
            int col = static_cast<int>((check_x) / cell_size) - margin_px / grid_px;
            int row = static_cast<int>((check_y) / cell_size) - margin_px / grid_px;
        
            // 범위 검사
            if (row >= 0 && row < grid_rows && col >= 0 && col < grid_cols) {
                int people = people_matrix_[row][col];
                if (label == "front") front_congestion = calculate_congestion(people);
                else if (label == "left") left_congestion =  calculate_congestion(people);
                else if (label == "right") right_congestion =  calculate_congestion(people);
            } else {
                // out of bounds → -1 유지
                if (label == "front") front_congestion = -1;
                else if (label == "left") left_congestion = -1;
                else if (label == "right") right_congestion = -1;
            }
        }

        geometry_msgs::msg::Vector3 msg;
        msg.x = static_cast<double>(front_congestion);
        msg.y = static_cast<double>(left_congestion);
        msg.z = static_cast<double>(right_congestion);
        congestion_pub_->publish(msg);
        // 디버깅 출력
        RCLCPP_INFO(this->get_logger(), "혼잡도 → front: %d, left: %d, right: %d",
                    front_congestion, left_congestion, right_congestion);
    }


    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr congestion_pub_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr direction_pub_;      
        
    bool pose_received_ = false;
    bool eta_received_;
    bool map_received_;
    bool people_loaded_;
    int64_t last_direction_ = 999;  // 초기에는 존재하지 않는 값으로 설정

    geometry_msgs::msg::Pose current_pose_;
    nav_msgs::msg::MapMetaData map_info_;
    
    std::vector<std::vector<int>> people_matrix_;
    std::vector<double> starting_speed_;
    std::vector<std::vector<double>> velocity_change_;
    std::vector<std::vector<double>> distance_change_;
    std::vector<double> starting_distance_change_;

    std::ofstream debug_out_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EtaOnceSubscriber>());
    rclcpp::shutdown();
    return 0;
}