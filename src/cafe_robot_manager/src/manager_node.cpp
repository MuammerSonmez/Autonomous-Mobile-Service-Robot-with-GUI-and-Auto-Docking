#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

class CafeManagerNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    CafeManagerNode() : Node("cafe_manager_node")
    {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        timer_ = this->create_wall_timer(
            1000ms, std::bind(&CafeManagerNode::send_test_goal, this));
    }

    void send_test_goal()
    {
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 is not work.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Nav2 ready, sent TEST pose...");

        auto goal_msg = NavigateToPose::Goal();
        
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        
        // Konum (X, Y)
        goal_msg.pose.pose.position.x = 9.841; 
        goal_msg.pose.pose.position.y = 2.746;
        goal_msg.pose.pose.position.z = 0.0;

        // Yonelim (Quaternion Z, W)
        goal_msg.pose.pose.orientation.z = -0.106;
        goal_msg.pose.pose.orientation.w = 0.994;

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            std::bind(&CafeManagerNode::result_callback, this, std::placeholders::_1);
        
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void result_callback(const GoalHandleNav::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "succes");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "canceld");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "uknown");
                break;
        }
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CafeManagerNode>());
    rclcpp::shutdown();
    return 0;
}