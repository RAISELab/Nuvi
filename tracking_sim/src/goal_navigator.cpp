#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <fstream>
#include <sstream>
#include <string>

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GoalNavigator : public rclcpp::Node
{
public:
    GoalNavigator() : Node("goal_navigator")
    {
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // íŒŒì¼ì—ì„œ ëª©í‘œ ì¢Œí‘œ ì½ê¸°
        if (!readGoalFromFile("/home/min/turtlebot3_ws/src/tracking_sim/config/destinations.txt", target_pose_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read goal from file.");
            return;
        }

        // 1ë‹¨ê³„: ëª©í‘œ ìœ„ì¹˜ë¡œ ì´ë™
        sendGoal(target_pose_);

        // 2ë‹¨ê³„: ì›ì (Station)ìœ¼ë¡œ ë³µê·€
        sendGoal(getStationPose());
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    geometry_msgs::msg::PoseStamped target_pose_;

    // íŒŒì¼ì—ì„œ ëª©í‘œë¥¼ ì½ëŠ” í•¨ìˆ˜
    bool readGoalFromFile(const std::string &filename, geometry_msgs::msg::PoseStamped &goal_pose)
    {
        std::ifstream file("/home/min/turtlebot3_ws/src/tracking_sim/config/destinations.txt");
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", filename.c_str());
            return false;
        }

        std::string line;
        if (std::getline(file, line))
        {
            std::istringstream ss(line);
            char comma;
            goal_pose.header.frame_id = "map"; // ì§€ë„ ì¢Œí‘œê³„ ì‚¬ìš©
            goal_pose.header.stamp = this->get_clock()->now();

            // í…ìŠ¤íŠ¸ íŒŒì¼ í˜•ì‹: x, y, z, orientation_x, orientation_y, orientation_z, orientation_w
            ss >> goal_pose.pose.position.x >> comma
               >> goal_pose.pose.position.y >> comma
               >> goal_pose.pose.position.z >> comma
               >> goal_pose.pose.orientation.x >> comma
               >> goal_pose.pose.orientation.y >> comma
               >> goal_pose.pose.orientation.z >> comma
               >> goal_pose.pose.orientation.w;

            file.close();
            return true;
        }

        file.close();
        return false;
    }

    // ì›ì (Station)ìœ¼ë¡œ ëŒì•„ì˜¤ëŠ” ì¢Œí‘œë¥¼ ì„¤ì •í•˜ëŠ” í•¨ìˆ˜
    geometry_msgs::msg::PoseStamped getStationPose()
    {
        geometry_msgs::msg::PoseStamped station_pose;
        station_pose.header.frame_id = "map";
        station_pose.header.stamp = this->get_clock()->now();

        // ì›ì  ì¢Œí‘œ ì„¤ì • (ì˜ˆì‹œ)
        station_pose.pose.position.x = 0.0;
        station_pose.pose.position.y = 0.0;
        station_pose.pose.position.z = 0.0;
        station_pose.pose.orientation.x = 0.0;
        station_pose.pose.orientation.y = 0.0;
        station_pose.pose.orientation.z = 0.0;
        station_pose.pose.orientation.w = 1.0;

        return station_pose;
    }

    // ëª©í‘œë¥¼ ì „ì†¡í•˜ê³  ê²°ê³¼ë¥¼ ëŒ€ê¸°í•˜ëŠ” í•¨ìˆ˜
    void sendGoal(const geometry_msgs::msg::PoseStamped &goal_pose)
    {
        if (!client_ptr_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        RCLCPP_INFO(this->get_logger(), "Sending goal: [x: %.2f, y: %.2f]", 
                    goal_pose.pose.position.x, goal_pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            }
            else if (result.code == rclcpp_action::ResultCode::ABORTED)
            {
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            }
            else if (result.code == rclcpp_action::ResultCode::CANCELED)
            {
                RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            }
        };

        auto future_goal_handle = client_ptr_->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_goal_handle) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Send goal call failed");
            return;
        }

        auto goal_handle = future_goal_handle.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }

        // ëª©í‘œ ë„ë‹¬ê¹Œì§€ ëŒ€ê¸°
        auto future_result = client_ptr_->async_get_result(goal_handle);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalNavigator>());
    rclcpp::shutdown();
    return 0;
}