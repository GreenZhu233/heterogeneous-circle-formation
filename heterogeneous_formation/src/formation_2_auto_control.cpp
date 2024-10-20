#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "formation_interfaces/action/move_quad.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "formation_interfaces/srv/update_center.hpp"
#include <map>
#include <chrono>

using Odom = nav_msgs::msg::Odometry;
using MoveQuad = formation_interfaces::action::MoveQuad;
using GoalHandle = rclcpp_action::ClientGoalHandle<MoveQuad>;
using std::placeholders::_1;
using std::placeholders::_2;

class Robot
{
public:
    double x, y, z;
    int id;
    std::string ns;
    Robot(int id, std::string ns)
    {
        this->id = id;
        this->ns = ns;
        x = y = z = 0.0;
    }
    void odom_callback(const Odom::SharedPtr msg)
    {
        x = msg->pose.pose.position.x;
        y = msg->pose.pose.position.y;
        z = msg->pose.pose.position.z;
    }
};

class HeterogeneousFormation2AutoControl: public rclcpp::Node
{
public:
    int num;
    std::vector<double> uuv_center;
    double uuv_radius;
    double uuv_C1, uuv_C2, uuv_Lambda, uuv_Gamma;
    std::vector<double> quad_center;
    double quad_radius;
    double quad_C1, quad_C2, quad_Lambda, quad_Gamma;
    std::string quad_control_node_name;
    std::string uuv_control_node_name;
    rclcpp::AsyncParametersClient::SharedPtr quad_param_client;
    rclcpp::AsyncParametersClient::SharedPtr uuv_param_client;
    std::vector<std::shared_ptr<Robot>> uuvs;
    std::vector<std::shared_ptr<Robot>> quads;
    std::vector<std::shared_ptr<rclcpp::Subscription<Odom>>> uuv_odom_subs;
    std::vector<std::shared_ptr<rclcpp::Subscription<Odom>>> quad_odom_subs;
    std::vector<rclcpp_action::Client<MoveQuad>::SharedPtr> action_clients;
    int goal_achieved;
    rclcpp::Client<formation_interfaces::srv::UpdateCenter>::SharedPtr update_center_client;

    HeterogeneousFormation2AutoControl(std::string name): Node(name)
    {
        declare_parameter<int>("num", 3);
        declare_parameter<std::string>("quad_control_node_name", "/quadrotor_formation_lifecycle_node");
        declare_parameter<std::string>("uuv_control_node_name", "/rexrov_formation_lifecycle_node");
        declare_parameter<std::vector<double>>("uuv_center", std::vector<double>{0.0, 0.0, 0.0});
        declare_parameter<double>("uuv_radius", 1.0);
        declare_parameter<double>("uuv_C1", 0.4);
        declare_parameter<double>("uuv_C2", 4.0);
        declare_parameter<double>("uuv_Lambda", 0.5);
        declare_parameter<double>("uuv_Gamma", 0.01);
        declare_parameter<std::vector<double>>("quad_center", std::vector<double>{0.0, 0.0, 0.0});
        declare_parameter<double>("quad_radius", 1.0);
        declare_parameter<double>("quad_C1", 0.4);
        declare_parameter<double>("quad_C2", 1.0);
        declare_parameter<double>("quad_Lambda", 1.2);
        declare_parameter<double>("quad_Gamma", 0.01);

        get_parameter("num", num);
        get_parameter("quad_control_node_name", quad_control_node_name);
        get_parameter("uuv_control_node_name", uuv_control_node_name);
        get_parameter("uuv_center", uuv_center);
        get_parameter("uuv_radius", uuv_radius);
        get_parameter("uuv_C1", uuv_C1);
        get_parameter("uuv_C2", uuv_C2);
        get_parameter("uuv_Lambda", uuv_Lambda);
        get_parameter("uuv_Gamma", uuv_Gamma);
        get_parameter("quad_center", quad_center);
        get_parameter("quad_radius", quad_radius);
        get_parameter("quad_C1", quad_C1);
        get_parameter("quad_C2", quad_C2);
        get_parameter("quad_Lambda", quad_Lambda);
        get_parameter("quad_Gamma", quad_Gamma);

        uuvs.resize(num);
        quads.resize(num);
        uuv_odom_subs.resize(num);
        quad_odom_subs.resize(num);
        action_clients.resize(num);
        for(int i = 0; i < num; i++)
        {
            uuvs[i] = std::make_shared<Robot>(i, "/rexrov_" + std::to_string(i));
            quads[i] = std::make_shared<Robot>(i, "/quad_" + std::to_string(i));
            uuv_odom_subs[i] = create_subscription<Odom>(
                uuvs[i]->ns + "/pose_gt",
                1,
                std::bind(&Robot::odom_callback, uuvs[i], _1)
            );
            quad_odom_subs[i] = create_subscription<Odom>(
                quads[i]->ns + "/odom",
                1,
                std::bind(&Robot::odom_callback, quads[i], _1)
            );
            action_clients[i] = rclcpp_action::create_client<MoveQuad>(
                this,
                quads[i]->ns + "/position_control_action"
            );
        }
        quad_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, quad_control_node_name);
        uuv_param_client = std::make_shared<rclcpp::AsyncParametersClient>(this, uuv_control_node_name);
        update_center_client = create_client<formation_interfaces::srv::UpdateCenter>("update_quad_center");
    }

    bool send_goal(int id, MoveQuad::Goal goal)
    {
        if(!action_clients[id]->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Action server available, sending goal");
        auto send_goal_options = rclcpp_action::Client<MoveQuad>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&HeterogeneousFormation2AutoControl::result_callback, this, _1, id);
        action_clients[id]->async_send_goal(goal, send_goal_options);
        return true;
    }

    void result_callback(const GoalHandle::WrappedResult &result, int id)
    {
        switch(result.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "quadrotor %d: Goal succeeded", id);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_INFO(this->get_logger(), "quadrotor %d: Goal aborted", id);
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "quadrotor %d: Goal canceled", id);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "quadrotor %d: Unknown result code", id);
                break;
        }
        goal_achieved++;
    }
};

bool cmp(std::shared_ptr<Robot> probot1, std::shared_ptr<Robot> probot2, std::vector<double> center)
{
    double phi1 = atan2(probot1->y - center[1], probot1->x - center[0]);
    double phi2 = atan2(probot2->y - center[1], probot2->x - center[0]);
    if(phi1 < phi2) return true;
    else return false;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeterogeneousFormation2AutoControl>("heterogeneous_formation_2_auto_control_node");

    // drive quadrotors to upon the UUVs
    std::vector<double> center_1 = node->uuv_center;
    center_1[2] += 4.0;
    auto update_center_request = std::make_shared<formation_interfaces::srv::UpdateCenter::Request>();
    update_center_request->x = center_1[0];
    update_center_request->y = center_1[1];
    update_center_request->z = center_1[2];
    RCLCPP_INFO(node->get_logger(), "Updating quadrotor center to (%f, %f, %f)", center_1[0], center_1[1], center_1[2]);
    if(!node->update_center_client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Update center service not available");
    }
    auto update_center_future = node->update_center_client->async_send_request(update_center_request);
    rclcpp::spin_until_future_complete(node, update_center_future);
    if (update_center_future.get()->success) {
        RCLCPP_INFO(node->get_logger(), "Center update successful");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Center update failed");
    }

    // set parameters
    std::vector<rclcpp::Parameter> parameters;
    parameters.push_back(rclcpp::Parameter("C1", 0.2));
    parameters.push_back(rclcpp::Parameter("C2", node->uuv_C2));
    parameters.push_back(rclcpp::Parameter("Lambda", node->uuv_Lambda));
    parameters.push_back(rclcpp::Parameter("Gamma", node->uuv_Gamma));
    auto set_parameters_future = node->quad_param_client->set_parameters(parameters);
    rclcpp::spin_until_future_complete(node, set_parameters_future);
    parameters.clear();
    parameters.push_back(rclcpp::Parameter("C1", 0.2));
    parameters.push_back(rclcpp::Parameter("radius", node->quad_radius));
    set_parameters_future = node->uuv_param_client->set_parameters(parameters);
    rclcpp::spin_until_future_complete(node, set_parameters_future);
    sleep(10.0);

    // sort quadrotors and UUVs
    rclcpp::spin_some(node);
    std::sort(node->quads.begin(), node->quads.end(), std::bind(cmp, _1, _2, node->uuv_center));
    std::sort(node->uuvs.begin(), node->uuvs.end(), std::bind(cmp, _1, _2, node->uuv_center));

    // deactivate quadrotor formation control node
    auto lifecycle_client = node->create_client<lifecycle_msgs::srv::ChangeState>(node->quad_control_node_name + "/change_state");
    if (!lifecycle_client->wait_for_service(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(node->get_logger(), "Lifecycle service not available");
    }
    auto change_state_request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    change_state_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
    auto deactivate_future = lifecycle_client->async_send_request(change_state_request);
    rclcpp::spin_until_future_complete(node, deactivate_future);

    // steer quadrotors to land on the UUVs
    auto goal = MoveQuad::Goal();
    goal.is_relative_to_self = false;
    goal.is_relative_to_target = true;
    goal.position = {0.0, 0.0, 4.0};
    goal.error_tolerance = 0.3;
    goal.time_limit = 10.0;
    goal.turn_off_motors_after_reach = false;
    node->goal_achieved = 0;
    for(int i = 0; i < node->num; i++)
    {
        goal.target_odom_topic = node->uuvs[i]->ns + "/pose_gt";
        node->send_goal(node->quads[i]->id, goal);
    }
    while(node->goal_achieved < node->num)
    {
        rclcpp::spin_some(node);
    }
    goal.position = {0.0, 0.0, 0.9};
    goal.error_tolerance = 0.2;
    goal.turn_off_motors_after_reach = true;
    goal.time_limit = 30.0;
    node->goal_achieved = 0;
    for(int i = 0; i < node->num; i++)
    {
        goal.target_odom_topic = node->uuvs[i]->ns + "/pose_gt";
        node->send_goal(node->quads[i]->id, goal);
    }
    while(node->goal_achieved < node->num)
    {
        rclcpp::spin_some(node);
    }
    sleep(10.0);

    // relaunch quadrotors
    change_state_request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
    auto activate_future = lifecycle_client->async_send_request(change_state_request);
    rclcpp::spin_until_future_complete(node, activate_future);
    sleep(1.0);

    // reset parameters
    parameters.clear();
    parameters.push_back(rclcpp::Parameter("C1", node->quad_C1));
    parameters.push_back(rclcpp::Parameter("C2", node->quad_C2));
    parameters.push_back(rclcpp::Parameter("Lambda", node->quad_Lambda));
    parameters.push_back(rclcpp::Parameter("Gamma", node->quad_Gamma));
    parameters.push_back(rclcpp::Parameter("radius", node->quad_radius));
    set_parameters_future = node->quad_param_client->set_parameters(parameters);
    rclcpp::spin_until_future_complete(node, set_parameters_future);
    parameters.clear();
    parameters.push_back(rclcpp::Parameter("C1", node->uuv_C1));
    parameters.push_back(rclcpp::Parameter("radius", node->uuv_radius));
    set_parameters_future = node->uuv_param_client->set_parameters(parameters);
    rclcpp::spin_until_future_complete(node, set_parameters_future);

    rclcpp::shutdown();
    return 0;
}