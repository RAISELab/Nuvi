#include <chrono>
#include <thread> //thread add...
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// 이미지 토픽 관련
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

//custom service
#include "tetra_msgs/srv/parameter_read.hpp"
#include "tetra_msgs/srv/parameter_write.hpp"

//move mode service
#include "tetra_msgs/srv/set_move_mode.hpp"
#include "tetra_msgs/srv/linear_position_move.hpp"
#include "tetra_msgs/srv/angular_position_move.hpp"

extern "C"
{
  #include "drive_module.h"
  #include "dssp_rs232_drive_module.h"
}

using namespace std;

#define WHEEL_RADIUS    0.1018  //m
#define WHEEL_DISTANCE  0.4380  //m
#define TREAD_WIDTH     0.04    //m

using std::placeholders::_1;
using namespace std::chrono_literals;

pthread_t p_odom_loop_thread;

//serial
int com_port = 0;
char port[16] = {0,};
//calc
bool first;
double prev_coordinates[3] = {0.0, };
double coordinates[3] = {0.0, }; //x,y,theta
double dt = 0.0;
double velocity[3] = {0.0, };
//cmd_vel
double input_linear=0.0, input_angular = 0.0;
double control_linear=0.0, control_angular = 0.0;
double linear=0.0, angular = 0.0, bt_linear = 0.0, bt_angular = 0.0;
int m_old_accel_data = 50;
//Pose
double m_dX_pos = 0.0;
double m_dY_pos = 0.0;
double m_dTheta = 0.0;
int Reset = 0;
//bumper & emg
int m_bumper_data = 0;
int m_emg_state = 0;
//Error Code
int m_left_error_code = 0;
int m_right_error_code = 0;
//Position move
int m_iPOS_Y = 0;
int m_iPOS_Theta = 0;
bool bPosition_mode_flag = false;
//emg one time check flag
bool m_bCheck_emg = true;
//Joystick value
int joy_linear = 1.0;
int joy_angular = 1.0;
//Parameter data
int m_iParam = 0;
//ekf_localization
bool m_bEKF_option = false;
bool m_bForwardCheck = false;

//Joystick Enable & Disable
bool m_bFlag_joy_enable = false;

//------------------------------------------------------------------------------
// ROI 내에서 선을 검출하여 선의 각도(도 단위)와 ROI 시각화 이미지를 구하는 함수
// image         : 입력 이미지
// roi           : 관심 영역 (cv::Rect). roi.width 또는 roi.height가 0이면 전체 이미지 사용
// margin        : ROI 내 선 검출 시 여유 margin (픽셀)
// angle         : 검출된 선의 각도 (도 단위, 출력)
// image_with_line: 원본 이미지에 선과 ROI를 그린 결과 이미지 (출력)
// roi_visual    : 추출된 ROI 영역을 별도 창에 표시할 이미지 (출력)
// new_roi       : 검출된 선을 포함하는 바운딩 박스 (margin 확장 적용, 출력)
// 반환값 true: 선 검출 성공, false: 실패
bool extract_line_angle(const cv::Mat &image, const cv::Rect &roi, int margin,
                          double &angle, cv::Mat &image_with_line, cv::Mat &roi_visual, cv::Rect &new_roi)
{
    cv::Rect effective_roi = roi;
    if (effective_roi.width <= 0 || effective_roi.height <= 0)
        effective_roi = cv::Rect(0, 0, image.cols, image.rows);

    cv::Mat image_roi = image(effective_roi).clone();
    roi_visual = image_roi.clone();

    image_with_line = image.clone();
    cv::rectangle(image_with_line, effective_roi, cv::Scalar(255, 0, 0), 2);

    cv::Mat gray, blurred, edges;
    cv::cvtColor(image_roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5,5), 0);
    cv::Canny(blurred, edges, 50, 150, 3);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 50, 10);
    if (lines.empty())
        return false;

    cv::Vec4i longest_line;
    double max_length = 0;
    bool found_line = false;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        if (x1 < margin || x2 < margin || x1 > (effective_roi.width - margin) || x2 > (effective_roi.width - margin) ||
            y1 < margin || y2 < margin || y1 > (effective_roi.height - margin) || y2 > (effective_roi.height - margin))
        {
            continue;
        }
        double length = std::hypot(x2 - x1, y2 - y1);
        if (length > max_length) {
            max_length = length;
            longest_line = line;
            found_line = true;
        }
    }
    if (!found_line)
        return false;

    int x1_global = longest_line[0] + effective_roi.x;
    int y1_global = longest_line[1] + effective_roi.y;
    int x2_global = longest_line[2] + effective_roi.x;
    int y2_global = longest_line[3] + effective_roi.y;

    angle = std::atan2((y2_global - y1_global), (x2_global - x1_global)) * 180.0 / CV_PI;

    cv::line(image_with_line, cv::Point(x1_global, y1_global), cv::Point(x2_global, y2_global),
             cv::Scalar(0, 0, 255), 2);

    int x_min = std::max(std::min(x1_global, x2_global) - margin, 0);
    int y_min = std::max(std::min(y1_global, y2_global) - margin, 0);
    int x_max = std::min(std::max(x1_global, x2_global) + margin, image.cols);
    int y_max = std::min(std::max(y1_global, y2_global) + margin, image.rows);
    new_roi = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

    return true;
}

//------------------------------------------------------------------------------
// TETRA 노드 클래스 (ROS2)
class TETRA : public rclcpp::Node
{
public:
  TETRA() : Node("tetra"),
            baseline_line_set_(false),
            baseline_line_angle_(0.0),
            last_heading_error_(0.0)
  {
    // tf2_ros
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Publisher
    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS());
    bumper_publisher = this->create_publisher<std_msgs::msg::Int32>("bumper_data", rclcpp::SystemDefaultsQoS());
    emg_publisher = this->create_publisher<std_msgs::msg::Int32>("emg_state", rclcpp::SystemDefaultsQoS());
    left_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("left_error_code", rclcpp::SystemDefaultsQoS());
    right_error_code_publisher = this->create_publisher<std_msgs::msg::Int32>("right_error_code", rclcpp::SystemDefaultsQoS());

    // Subscription
    joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::SensorDataQoS(), std::bind(&TETRA::joyCallback, this, _1));
    cmd_vel_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SensorDataQoS(), std::bind(&TETRA::velCallback, this, _1));
    acc_subscriber = this->create_subscription<std_msgs::msg::Int32>("accel_vel", rclcpp::SystemDefaultsQoS(), std::bind(&TETRA::accelCallback, this, _1));
    pose_reset_subscriber = this->create_subscription<std_msgs::msg::Int32>("pose_reset", rclcpp::SystemDefaultsQoS(), std::bind(&TETRA::pose_resetCallback, this, _1));
    servo_on_subscriber = this->create_subscription<std_msgs::msg::Int32>("servo_on", rclcpp::SystemDefaultsQoS(), std::bind(&TETRA::servo_onCallback, this, _1));
    power_status_subscriber = this->create_subscription<std_msgs::msg::Int32>("power_status", rclcpp::SystemDefaultsQoS(), std::bind(&TETRA::power_statusCallback, this, _1));

    // 이미지 구독 (realsense_camera에서 발행하는 이미지 토픽)
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&TETRA::imageCallback, this, _1));

    // Service들 (생략: 기존 코드와 동일)
    parameter_read_srv = this->create_service<tetra_msgs::srv::ParameterRead>(
        "param_read_cmd", std::bind(&TETRA::Parameter_Read_Command, this, std::placeholders::_1, std::placeholders::_2));
    parameter_write_srv = this->create_service<tetra_msgs::srv::ParameterWrite>(
        "param_write_cmd", std::bind(&TETRA::Parameter_Write_Command, this, std::placeholders::_1, std::placeholders::_2));
    set_move_mode_srv = this->create_service<tetra_msgs::srv::SetMoveMode>(
        "mode_change_cmd", std::bind(&TETRA::Movemode_Change_Command, this, std::placeholders::_1, std::placeholders::_2));
    linear_position_move_srv = this->create_service<tetra_msgs::srv::LinearPositionMove>(
        "linear_move_cmd", std::bind(&TETRA::Linear_Move_Command, this, std::placeholders::_1, std::placeholders::_2));
    angular_position_move_srv = this->create_service<tetra_msgs::srv::AngularPositionMove>(
        "angular_move_cmd", std::bind(&TETRA::Angular_Move_Command, this, std::placeholders::_1, std::placeholders::_2));

    // PARAMETER
    this->declare_parameter("m_bEKF_option", false);
    m_bEKF_option_param = this->get_parameter("m_bEKF_option");
    m_bEKF_option = m_bEKF_option_param.as_bool();

    // 이미지 선 검출 관련 초기화
    line_roi_ = cv::Rect(0, 0, 0, 0);
    first_line_frame_ = true;
    prev_line_angle_ = 0.0;

    // 시작 시간 저장 (4초간 섞기 위해)
    start_time_ = this->now();

    // OpenCV 창 생성
    cv::namedWindow("Line", cv::WINDOW_NORMAL);
    // cv::namedWindow("ROI", cv::WINDOW_NORMAL);
  }

  ~TETRA()
  {
    cv::destroyAllWindows();
  }

  // Public 메서드 (예: SetMoveCommand 등)
  void SetMoveCommand(double fLinear_vel, double fAngular_vel)
  {
    double Left_Wheel_vel = 0.0;
    double Right_Wheel_vel = 0.0;
    double linearRotate = (fLinear_vel / (M_PI * (2.0 * WHEEL_RADIUS)));
    double angularRotate = ((WHEEL_DISTANCE / 2.0) / (2.0 * WHEEL_RADIUS)) * fAngular_vel;
    Left_Wheel_vel = (linearRotate - angularRotate) * 60.0;
    Right_Wheel_vel = (linearRotate + angularRotate) * 60.0;

    int iData1 = static_cast<int>(1000.0 * (WHEEL_RADIUS * 2.0 * M_PI * (Left_Wheel_vel / 60.0)));
    int iData2 = static_cast<int>(1000.0 * (WHEEL_RADIUS * 2.0 * M_PI * (Right_Wheel_vel / 60.0)));
    dssp_rs232_drv_module_set_velocity(iData1, iData2);
  }

  // 현재 시간 (odometry 계산용)
  rclcpp::Time current_time, last_time;
  // tf2_ros 브로드캐스터
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Public Publisher들
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bumper_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr emg_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr left_error_code_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr right_error_code_publisher;

  // Public 변수 (예: m_bEKF_option_param)
  rclcpp::Parameter m_bEKF_option_param;

  // 시작 시간와 이미지 기반 heading error (라디안)
  rclcpp::Time start_time_;

  bool baseline_line_set_;
  double baseline_line_angle_;
  double last_heading_error_;

  //-------------------------------------------------------------------------------
  // 이미지 콜백: 이미지에서 선을 검출하여 선 각도 계산 및 ROI 업데이트,
  // 그리고 heading error를 계산하여 저장 (노드 시작 후 4초간 적용)
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat cv_image;
    try {
      cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    double current_angle;
    cv::Mat image_with_line, roi_visual;
    cv::Rect new_roi;
    if (!extract_line_angle(cv_image, line_roi_, 20, current_angle, image_with_line, roi_visual, new_roi))
    {
      RCLCPP_WARN(this->get_logger(), "No valid line detected in ROI.");
      line_roi_ = cv::Rect(0, 0, 0, 0);
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Current line angle: %.2f°", current_angle);
      // baseline 설정: 첫 프레임에서 baseline_line_angle_ 저장
      if (!baseline_line_set_)
      {
        baseline_line_angle_ = current_angle;
        baseline_line_set_ = true;
        RCLCPP_INFO(this->get_logger(), "Baseline line angle set: %.2f°", baseline_line_angle_);
      }
      else
      {
        // heading error (도 단위) = 현재 line angle - baseline line angle
        double heading_error_deg = baseline_line_angle_ - current_angle;
        // radian으로 변환 후 저장
        last_heading_error_ = heading_error_deg * CV_PI / 180.0;
        RCLCPP_INFO(this->get_logger(), "Heading error from line: %.2f rad", last_heading_error_);
      }

      if (!first_line_frame_)
      {
        double angle_diff = std::abs(current_angle - prev_line_angle_);
        RCLCPP_INFO(this->get_logger(), "Angle diff: %.2f°", angle_diff);
        if (angle_diff > 30.0)
        {
          RCLCPP_WARN(this->get_logger(), "Angle difference too large (%.2f°); update ignored.", angle_diff);
        }
        else
        {
          prev_line_angle_ = current_angle;
          line_roi_ = new_roi;
        }
      }
      else
      {
        first_line_frame_ = false;
        prev_line_angle_ = current_angle;
        line_roi_ = new_roi;
        RCLCPP_INFO(this->get_logger(), "First frame line angle: %.2f°", current_angle);
      }
    }

    cv::imshow("Line", image_with_line);
    cv::imshow("ROI", roi_visual);
    cv::waitKey(1);
  }


  void velCallback(const geometry_msgs::msg::Twist::SharedPtr vel)
  {
    linear = vel->linear.x;
    angular = vel->angular.z;
  }

  void accelCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int m_iAccel_data = msg->data;
    if(m_old_accel_data != m_iAccel_data)
    {
      dssp_rs232_drv_module_set_parameter(6, m_iAccel_data);
      m_old_accel_data = m_iAccel_data;
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    if(joy->buttons[8])
    {
      m_bFlag_joy_enable = false;
    }
    if(joy->buttons[9])
    {
      m_bFlag_joy_enable = true;
    }

    if(m_bFlag_joy_enable)
    {
      if(joy->axes[4])
      {
        bt_angular += joy_angular * joy->axes[4];
      }
      if(joy->axes[5])
      {
        bt_linear += joy_linear * joy->axes[5];
      }
      if(joy->buttons[0])
      {
        joy_angular = max(joy_angular - 1.0, 0.0);
      }
      if(joy->buttons[1])
      {
        joy_linear = max(joy_linear - 1.0, 0.0);
      }
      if(joy->buttons[2])
      {
        joy_angular += 1.0;
      }
      if(joy->buttons[3])
      {
        joy_linear += 1.0;
      }
      if(joy->buttons[5])
      {
        bt_linear = 0;
        bt_angular = 0;
        joy_linear = 1.0;
        joy_angular = 1.0;
      }
      linear = (joy_linear * (double)joy->axes[1] + bt_linear) / 3.0;
      angular = ((double)joy_angular * (joy->axes[1] >= 0 ? joy->axes[2] : (joy->axes[2] * -1)) + bt_angular) / 3.0;
    }
  }

  void pose_resetCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    Reset = msg->data;
    if(Reset == 1)
    {
      dssp_rs232_drv_module_reset_odometry();
      Reset = 0;
    }
  }

  void servo_onCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int m_idata = msg->data;
    if(m_idata == 1)
    {
      bPosition_mode_flag = false;
      dssp_rs232_drv_module_set_velocitymode();
      usleep(10000);
      dssp_rs232_drv_module_set_servo(1);
      m_idata = 0;
    }
    else if(m_idata == 2)
    {
      dssp_rs232_drv_module_set_servo(0);
      m_idata = 0;
    }
    else if(m_idata == 3)
    {
      bPosition_mode_flag = true;
      dssp_rs232_drv_module_set_positionmode();
      usleep(10000);
      dssp_rs232_drv_module_set_servo(1);
      m_idata = 0;
    }
  }

  void power_statusCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int m_idata = msg->data;
    if(m_idata == -10)
    {
      dssp_rs232_drv_module_set_servo(0);
      printf("[Error]: Power Board Error !!! \n");
    }
  }

  // Service 콜백들 (Parameter_Read_Command, Parameter_Write_Command, Movemode_Change_Command,
  // Linear_Move_Command, Angular_Move_Command)
  bool Parameter_Read_Command(
      const std::shared_ptr<tetra_msgs::srv::ParameterRead::Request> request,
      const std::shared_ptr<tetra_msgs::srv::ParameterRead::Response> response)
  {
    bool bResult = false;
    dssp_rs232_drv_module_read_parameter(request->num, &m_iParam);
    response->data = m_iParam;
    bResult = true;
    response->command_result = bResult;
    return true;
  }

  bool Parameter_Write_Command(
      const std::shared_ptr<tetra_msgs::srv::ParameterWrite::Request> request,
      const std::shared_ptr<tetra_msgs::srv::ParameterWrite::Response> response)
  {
    bool bResult = false;
    dssp_rs232_drv_module_set_parameter(request->num, request->data);
    bResult = true;
    response->command_result = bResult;
    return true;
  }

  bool Movemode_Change_Command(
      const std::shared_ptr<tetra_msgs::srv::SetMoveMode::Request> request,
      const std::shared_ptr<tetra_msgs::srv::SetMoveMode::Response> response)
  {
    bool bResult = false;
    if(request->mode == 1)
    {
      dssp_rs232_drv_module_set_servo(0);
      usleep(10000);
      dssp_rs232_drv_module_set_positionmode();
      usleep(10000);
      dssp_rs232_drv_module_set_servo(1);
      usleep(10000);
    }
    else
    {
      dssp_rs232_drv_module_set_servo(0);
      usleep(10000);
      dssp_rs232_drv_module_set_velocitymode();
      usleep(10000);
      dssp_rs232_drv_module_set_servo(1);
      usleep(10000);
    }
    bResult = true;
    response->command_result = bResult;
    return true;
  }

  bool Linear_Move_Command(
      const std::shared_ptr<tetra_msgs::srv::LinearPositionMove::Request> request,
      const std::shared_ptr<tetra_msgs::srv::LinearPositionMove::Response> response)
  {
    bool bResult = false;
    dssp_rs232_drv_module_set_position(0, 0, request->linear_position);
    bResult = true;
    response->command_result = bResult;
    return true;
  }

  bool Angular_Move_Command(
      const std::shared_ptr<tetra_msgs::srv::AngularPositionMove::Request> request,
      const std::shared_ptr<tetra_msgs::srv::AngularPositionMove::Response> response)
  {
    bool bResult = false;
    dssp_rs232_drv_module_set_position(2, 0, request->angular_degree);
    bResult = true;
    response->command_result = bResult;
    return true;
  }

public:
  // Public 변수들 (필요시)
  // ...
  
private:
  // 이미지 구독 관련 멤버 변수
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  cv::Rect line_roi_;

  bool first_line_frame_;
  double prev_line_angle_;



  // 기존 Subscription 및 Service 멤버 변수
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr acc_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr pose_reset_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr servo_on_subscriber;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr power_status_subscriber;

  rclcpp::Service<tetra_msgs::srv::ParameterRead>::SharedPtr parameter_read_srv;
  rclcpp::Service<tetra_msgs::srv::ParameterWrite>::SharedPtr parameter_write_srv;
  rclcpp::Service<tetra_msgs::srv::SetMoveMode>::SharedPtr set_move_mode_srv;
  rclcpp::Service<tetra_msgs::srv::LinearPositionMove>::SharedPtr linear_position_move_srv;
  rclcpp::Service<tetra_msgs::srv::AngularPositionMove>::SharedPtr angular_position_move_srv;
};

//------------------------------------------------------------------------------
// Main Loop
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto tetra = std::make_shared<TETRA>();

  tetra->current_time = tetra->now();
  tetra->last_time = tetra->current_time;
  first = true;
  for(int i = 0; i < 3; i++)
  {
    prev_coordinates[i] = 0;
    coordinates[i] = 0;
  }

  rclcpp::WallRate loop_rate(30); // 30Hz

  sprintf(port, "/dev/ttyUSB0");
  if(dssp_rs232_drv_module_create(port, 200) == 0)
  {
    printf("TETRA_DS_rs232 Port Open Success\n");
  }
  else
  {
    printf("TETRA_DS_rs232 Port Open Error!\n");
    return -1;
  }

  dssp_rs232_drv_module_set_drive_err_reset();
  usleep(10000);
  dssp_rs232_drv_module_set_parameter(6, 50);
  usleep(10000);
  dssp_rs232_drv_module_set_velocitymode();
  usleep(10000);
  dssp_rs232_drv_module_set_servo(1);
  usleep(10000);
  dssp_rs232_drv_module_reset_odometry();
  usleep(10000);
  bool m_bflag_emg = false;
  
  printf("□■■■■■■■□■■■■■■□□■■■■■■■□■■■■■■□□□□□□■□□□□\n");
  printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□□□■□□□□\n");
  printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□□■□■□□□\n");
  printf("□□□□■□□□□■■■■■■□□□□□■□□□□■■■■■■□□□□■□□□■□□\n");
  printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□□■■■■■□□\n");
  printf("□□□□■□□□□■□□□□□□□□□□■□□□□■□□□□□■□□■□□□□□■□\n");
  printf("□□□□■□□□□■■■■■■□□□□□■□□□□■□□□□□■□□■□□□□□■□\n");                 

  while (rclcpp::ok())
  {
    rclcpp::spin_some(tetra);
    
    input_linear  = linear;
    input_angular = angular;

    if(linear > 0)
      m_bForwardCheck = true;
    else
      m_bForwardCheck = false;

    if(m_bForwardCheck)
    {
      if(input_linear > control_linear)
      {
        control_linear = min(input_linear, control_linear + 0.01);
      }
      else if(input_linear < control_linear)
      {
        control_linear = max(input_linear, control_linear - 0.05);
      }
      else
      {
        control_linear = input_linear;
      }
    }
    else
    {
      if(input_linear < control_linear)
      {
        control_linear = max(input_linear, control_linear - 0.01);
        if(control_linear > 0)
        {
          control_linear = max(input_linear, control_linear - 0.05);
        }
      }
      else if(input_linear > control_linear)
      {
        control_linear = min(input_linear, control_linear + 0.05);
      }
      else
      {
        control_linear = input_linear;
      }
    }
    control_angular = input_angular;

    std_msgs::msg::Int32 emg_state;
    emg_state.data = m_emg_state;
    tetra->emg_publisher->publish(emg_state);

    std_msgs::msg::Int32 bumper_data;
    bumper_data.data = m_bumper_data;
    tetra->bumper_publisher->publish(bumper_data);

    std_msgs::msg::Int32 left_error_code;
    std_msgs::msg::Int32 right_error_code;
    left_error_code.data = m_left_error_code;
    right_error_code.data = m_right_error_code;
    tetra->left_error_code_publisher->publish(left_error_code);
    tetra->right_error_code_publisher->publish(right_error_code);

    dssp_rs232_drv_module_read_odometry(&m_dX_pos, &m_dY_pos, &m_dTheta);
    coordinates[0] = (m_dX_pos / 1000.0);
    coordinates[1] = (m_dY_pos / 1000.0);
    coordinates[2] = m_dTheta * (M_PI/1800.0);

    if(!bPosition_mode_flag)
    {
      tetra->SetMoveCommand(control_linear, control_angular);
      dssp_rs232_drv_module_read_bumper_emg(&m_bumper_data, &m_emg_state, &m_left_error_code, &m_right_error_code);
    }

    if(m_left_error_code != 48 || m_right_error_code != 48)
    {
      printf("[Motor Driver Error] Left Error Code: %d \n", m_left_error_code);
      printf("[Motor Driver Error] Right Error Code: %d \n", m_right_error_code);
      dssp_rs232_drv_module_set_drive_err_reset();
      usleep(1000);
      dssp_rs232_drv_module_set_servo(1);
    }

    if(m_emg_state)
    {
      if(m_bflag_emg)
      {
        dssp_rs232_drv_module_set_servo(0);
        usleep(1000);
        dssp_rs232_drv_module_set_drive_err_reset();
        usleep(1000);
        m_bflag_emg = false;
      }
    }
    else
    {
      if(!m_bflag_emg)
      {
        dssp_rs232_drv_module_set_servo(1);
        m_bflag_emg = true;
      }
    }

    if(first) 
    {
      tetra->current_time = tetra->now();
      tetra->last_time = tetra->current_time;
      first = false;
    }
    else 
    {
      tetra->current_time = tetra->now();
      dt = (tetra->current_time - tetra->last_time).seconds();
      for(int i = 0; i < 3; i++)
      {
        velocity[i] = (coordinates[i] - prev_coordinates[i]) / dt;
        prev_coordinates[i] = coordinates[i];
      }

      double odom_heading = coordinates[2]; // tetra에서 계산한 odometry heading (radian)
      double final_heading;

      // ------------------------detection & fusion params--------------------------

      if ((tetra->now() - tetra->start_time_).seconds() < 10.0)
      {
        final_heading = (odom_heading + tetra->last_heading_error_) / 2.0;
        //RCLCPP_INFO(tetra->get_logger(), "odom_heading: %.2f rad, last_heading_error: %.2f rad", odom_heading, tetra->last_heading_error_);
      }
      else
      {
        final_heading = odom_heading;
      }

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, final_heading);

      if(!m_bEKF_option)
      {
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = tetra->current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
        odom_trans.transform.translation.x = coordinates[0];
        odom_trans.transform.translation.y = coordinates[1];
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();
        tetra->tf_broadcaster_->sendTransform(odom_trans);
      }

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = tetra->current_time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.pose.pose.position.x = coordinates[0];
      odom.pose.pose.position.y = coordinates[1];
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom.twist.twist.linear.x = velocity[0];
      odom.twist.twist.linear.y = velocity[1];
      odom.twist.twist.linear.z = 0.0;
      odom.twist.twist.angular.z = velocity[2];
      tetra->odom_publisher->publish(odom);

      tetra->last_time = tetra->current_time;
    }

    loop_rate.sleep();
  }

  dssp_rs232_drv_module_set_servo(0);
  printf("Servo Off \n");
  usleep(10000);
  dssp_rs232_drv_module_destroy();
  printf("RS232 Disconnect \n");

  rclcpp::shutdown();
  return 0;
}
