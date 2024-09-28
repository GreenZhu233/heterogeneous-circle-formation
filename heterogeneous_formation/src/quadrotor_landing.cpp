#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "formation_interfaces/action/move_quad.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <string>

using std::placeholders::_1;

class PositionControlClient: public rclcpp::Node
{
public:
    using MoveQuad = formation_interfaces::action::MoveQuad;
    using GoalHandle = rclcpp_action::ClientGoalHandle<MoveQuad>;
    std::vector<double> uuv_pos{0, 0, 0};

    explicit PositionControlClient(std::string name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node(name, options), goal_done_(false)
    {
        this->declare_parameter("action_service_name", "/position_control");
        this->declare_parameter("uuv_odom_topic", "/rexrov_0/odom");
        std::string action_service_name = this->get_parameter("action_service_name").as_string();
        std::string uuv_odom_topic = this->get_parameter("uuv_odom_topic").as_string();
        this->action_client = rclcpp_action::create_client<MoveQuad>(
            this,
            action_service_name
        );
        this->uuv_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            uuv_odom_topic,
            1,
            std::bind(&PositionControlClient::uuv_odom_callback, this, _1)
        );
    }

    bool send_goal(MoveQuad::Goal goal)
    {
        this->goal_done_ = false;
        if(!this->action_client->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Action server available, sending goal");
        auto send_goal_options = rclcpp_action::Client<MoveQuad>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&PositionControlClient::goal_response_callback, this, _1);
        send_goal_options.result_callback = std::bind(&PositionControlClient::result_callback, this, _1);
        this->action_client->async_send_goal(goal, send_goal_options);
        return true;
    }

    bool is_goal_done()
    {
        return this->goal_done_;
    }

private:
    rclcpp_action::Client<MoveQuad>::SharedPtr action_client;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr uuv_odom_sub;
    bool goal_done_;

    void uuv_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->uuv_pos[0] = msg->pose.pose.position.x;
        this->uuv_pos[1] = msg->pose.pose.position.y;
        this->uuv_pos[2] = msg->pose.pose.position.z;
    }

    void goal_response_callback(const GoalHandle::SharedPtr &goal_handle)
    {
        if(!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandle::WrappedResult &result)
    {
        this->goal_done_ = true;
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "Goal aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        std::vector<double>pos(result.result->final_position);
        RCLCPP_INFO(this->get_logger(), "final position: %g, %g, %g", pos[0], pos[1], pos[2]);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionControlClient>("position_control_client");
    auto goal = PositionControlClient::MoveQuad::Goal();

    sleep(5);   // wait for me to turn on gazebo gui

    // lift off
    goal.position = {0.0, 0.0, 4.0};
    goal.is_relative_to_self = true;
    goal.error_tolerance = 0.5;
    if(!node->send_goal(goal))
    {
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Lifting off goal sent");
    while(rclcpp::ok() && !node->is_goal_done())
    {
        rclcpp::spin_some(node);
    }

    // move to above the uuv
    goal.position = {0.0, 0.0, 4.0};
    goal.is_relative_to_self = false;
    goal.is_relative_to_target = true;
    goal.error_tolerance = 0.3;
    goal.time_limit = 30.0;
    if(!node->send_goal(goal))
    {
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Moving to above the uuv goal sent");
    while(rclcpp::ok() && !node->is_goal_done())
    {
        rclcpp::spin_some(node);
    }

    // touch town
    goal.position = {0.0, 0.0, 0.9};
    goal.error_tolerance = 0.2;
    goal.turn_off_motors_after_reach = true;
    if(!node->send_goal(goal))
    {
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Touch down goal sent");
    while(rclcpp::ok() && !node->is_goal_done())
    {
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}