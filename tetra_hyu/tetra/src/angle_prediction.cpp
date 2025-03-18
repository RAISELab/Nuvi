#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

// ROI 내에서 선 검출 및 각도 계산 함수
// image         : 입력 이미지
// roi           : 관심 영역 (cv::Rect). roi.width 또는 roi.height가 0이면 전체 이미지 사용
// margin        : ROI 내 선 검출 시 여유 margin (픽셀)
// angle         : 검출된 선의 각도 (도 단위, 출력)
// image_with_line: 원본 이미지에 선과 ROI를 그린 결과 이미지 (출력)
// roi_visual    : 추출된 ROI 영역을 별도 창에 표시할 이미지 (출력)
// new_roi       : 검출된 선을 포함하는 바운딩 박스 (margin 확장 적용, 출력)
bool extract_line_angle(const cv::Mat &image, const cv::Rect &roi, int margin,
                          double &angle, cv::Mat &image_with_line, cv::Mat &roi_visual, cv::Rect &new_roi)
{
    // roi가 비어있으면 전체 이미지를 사용
    cv::Rect effective_roi = roi;
    if (effective_roi.width <= 0 || effective_roi.height <= 0)
        effective_roi = cv::Rect(0, 0, image.cols, image.rows);

    // ROI 영역 추출 및 시각화용 이미지 생성
    cv::Mat image_roi = image(effective_roi).clone();
    roi_visual = image_roi.clone();

    // ROI 영역을 원본 이미지에 표시 (디버깅용)
    image_with_line = image.clone();
    cv::rectangle(image_with_line, effective_roi, cv::Scalar(255, 0, 0), 2);

    // ROI 내 전처리: 그레이스케일 변환, Gaussian Blur, Canny Edge Detection
    cv::Mat gray, blurred, edges;
    cv::cvtColor(image_roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5,5), 0);
    cv::Canny(blurred, edges, 50, 150, 3);

    // Probabilistic Hough Transform으로 선 검출
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI/180, 50, 50, 10);
    if (lines.empty())
        return false;

    // ROI 내부(여유 margin 내)에 완전히 포함되는 선들 중 가장 긴 선 선택
    cv::Vec4i longest_line;
    double max_length = 0;
    bool found_line = false;
    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i line = lines[i];
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        // ROI 내부의 유효 범위: margin을 고려하여 너무 가장자리 근처의 선은 무시
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

    // ROI 내 좌표를 원본 이미지 좌표로 보정
    int x1_global = longest_line[0] + effective_roi.x;
    int y1_global = longest_line[1] + effective_roi.y;
    int x2_global = longest_line[2] + effective_roi.x;
    int y2_global = longest_line[3] + effective_roi.y;

    // 선의 각도 계산 (글로벌 좌표 기준, 도 단위)
    angle = std::atan2((y2_global - y1_global), (x2_global - x1_global)) * 180.0 / CV_PI;

    // 원본 이미지에 검출된 선 그리기
    cv::line(image_with_line, cv::Point(x1_global, y1_global), cv::Point(x2_global, y2_global), 
             cv::Scalar(0, 0, 255), 2);

    // 검출된 선을 포함하는 바운딩 박스(여유 margin 적용)를 새로운 ROI로 업데이트 (이미지 범위 내로 클리핑)
    int x_min = std::max(std::min(x1_global, x2_global) - margin, 0);
    int y_min = std::max(std::min(y1_global, y2_global) - margin, 0);
    int x_max = std::min(std::max(x1_global, x2_global) + margin, image.cols);
    int y_max = std::min(std::max(y1_global, y2_global) + margin, image.rows);
    new_roi = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);

    return true;
}

class LineAngleCalculator
{
public:
    LineAngleCalculator(ros::NodeHandle &nh)
      : it_(nh), first_frame_(true), prev_angle_(0.0)
    {
        // 구독할 이미지 토픽 (예: /camera/color/image_raw)
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, &LineAngleCalculator::imageCallback, this);

        // 창 생성 (OpenCV)
        cv::namedWindow("Line", cv::WINDOW_NORMAL);
        cv::namedWindow("ROI", cv::WINDOW_NORMAL);
    }

    ~LineAngleCalculator()
    {
        cv::destroyAllWindows();
    }

private:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Rect roi_;  // 현재 ROI (초기에는 전체 이미지 사용을 위해 비어있는 rect)
    bool first_frame_;
    double prev_angle_;

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv::Mat cv_image;
        try {
            cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        double current_angle;
        cv::Mat image_with_line, roi_visual;
        cv::Rect new_roi;
        // roi_가 비어있으면 cv::Rect(0,0,0,0) 전달 → extract_line_angle에서 전체 이미지 사용
        if (!extract_line_angle(cv_image, roi_, 20, current_angle, image_with_line, roi_visual, new_roi)) {
            ROS_WARN("ROI 내에서 선을 검출하지 못했습니다.");
            // ROI 내 검출 실패 시 전체 이미지로 재설정
            roi_ = cv::Rect(0,0,0,0);
        } else {
            ROS_INFO("현재 프레임 선 각도: %.2f°", current_angle);
            if (!first_frame_) {
                double angle_diff = std::abs(current_angle - prev_angle_);
                ROS_INFO("두 프레임 간 선 각도 차이: %.2f°", angle_diff);
                if (angle_diff > 30.0) {
                    ROS_WARN("각도 변화(%.2f°)가 30°를 초과하여 이번 업데이트를 무시합니다.", angle_diff);
                } else {
                    prev_angle_ = current_angle;
                    roi_ = new_roi;  // 업데이트된 ROI로 갱신
                }
            } else {
                first_frame_ = false;
                prev_angle_ = current_angle;
                roi_ = new_roi;
                ROS_INFO("첫 프레임 선 각도: %.2f°", current_angle);
            }
        }

        // 결과 이미지들을 화면에 표시
        cv::imshow("Line", image_with_line);
        cv::imshow("ROI", roi_visual);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_angle_calculator");
    ros::NodeHandle nh;
    LineAngleCalculator lac(nh);
    ros::spin();
    return 0;
}
