#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>

class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher() : Node("goal_publisher")
    {
        // "goal_pose" 토픽에 발행하는 퍼블리셔 생성
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

        // 터미널 인자로 "robotics_major_room"을 받으면 특정 목표를 설정
        this->declare_parameter<std::string>("goal", "robotics_major_room");
        std::string goal = this->get_parameter("goal").as_string();

        // "robotics_major_room"에 대한 좌표 설정
        if (goal == "robotics_major_room")
        {
            geometry_msgs::msg::PoseStamped goal_msg;
            goal_msg.header.stamp = this->get_clock()->now();
            goal_msg.header.frame_id = "base_footprint";  // 좌표계 설정

            goal_msg.pose.position.x = 18.753201074417106;
            goal_msg.pose.position.y = -0.6360023858710492;
            goal_msg.pose.position.z = 0.0;

            goal_msg.pose.orientation.x = 0.0;
            goal_msg.pose.orientation.y = 0.0;
            goal_msg.pose.orientation.z = -0.5794401414134441;
            goal_msg.pose.orientation.w = 0.8150147989569072;

            // 메시지를 "goal_pose" 토픽에 발행
            publisher_->publish(goal_msg);
            RCLCPP_INFO(this->get_logger(), "Published goal pose: [x: %.2f, y: %.2f, z: %.2f, orientation: (%.2f, %.2f, %.2f, %.2f)]",
                        goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z,
                        goal_msg.pose.orientation.x, goal_msg.pose.orientation.y, goal_msg.pose.orientation.z,
                        goal_msg.pose.orientation.w);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown goal '%s'. No goal published.", goal.c_str());
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}
