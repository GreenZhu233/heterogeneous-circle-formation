#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "formation_interfaces/action/move_quad.hpp"
#include "quadrotor_control/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench.hpp"

#include <string>
#include <thread>
#include <vector>
#include <cmath>
#include <atomic>
#include <map>

struct IntegratedError
{
    double sum;

    IntegratedError()
    {
        sum = 0;
    }

    double getvalue()
    {
        return sum;
    }

    void push(double value)
    {
        if(sum * value < 0)
        {
            sum = value;
        }
        else
        {
            sum += value;
        }
    }
};

class PositionControlServer: public rclcpp::Node
{
public:
    using MoveQuad = formation_interfaces::action::MoveQuad;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveQuad>;

    explicit PositionControlServer(std::string name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node(name, options)
    {
        using namespace std::placeholders;
        this->declare_parameter("G", 1.316*9.8);
        this->declare_parameter("self_odom_topic", "odom");
        this->declare_parameter("force_topic", "gazebo_ros_force");
        this->declare_parameter("action_service_name", "position_control_action");
        this->declare_parameter("pos_p", 1.0);
        this->declare_parameter("pos_i", 0.005);
        this->declare_parameter("pos_d", 2.2);
        this->declare_parameter("rot_p", std::vector<double>{0.04, 0.04, 0.08});
        this->declare_parameter("rot_d", std::vector<double>{0.1, 0.1, 0.2});
        this->param_callback_handle = this->add_on_set_parameters_callback(
            std::bind(&PositionControlServer::param_callback, this, _1));

        this->G = this->get_parameter("G").as_double();
        this->pos_p = this->get_parameter("pos_p").as_double();
        this->pos_i = this->get_parameter("pos_i").as_double();
        this->pos_d = this->get_parameter("pos_d").as_double();
        this->rot_p = this->get_parameter("rot_p").as_double_array();
        this->rot_d = this->get_parameter("rot_d").as_double_array();
        std::string self_odom_topic = this->get_parameter("self_odom_topic").as_string();
        std::string force_topic = this->get_parameter("force_topic").as_string();
        std::string action_service_name = this->get_parameter("action_service_name").as_string();
        this->action_server = rclcpp_action::create_server<MoveQuad>(
            this,
            action_service_name,
            std::bind(&PositionControlServer::handle_goal, this, _1, _2),
            std::bind(&PositionControlServer::handle_cancel, this, _1),
            std::bind(&PositionControlServer::handle_accepted, this, _1)
        );

        this->callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = this->callback_group;
        this->self_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            self_odom_topic,
            10,
            std::bind(&PositionControlServer::self_odom_callback, this, _1),
            sub_options
        );

        this->force_pub = this->create_publisher<geometry_msgs::msg::Wrench>(
            force_topic,
            10
        );
    }

private:
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr self_odom_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tracking_target_odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_pub;
    rclcpp_action::Server<MoveQuad>::SharedPtr action_server;
    double G;
    double pos_p, pos_i, pos_d;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
    std::vector<double> rot_p, rot_d;
    std::atomic<double> position[3];
    std::atomic<double> tracking_target_position[3];
    std::atomic<double> sim_time;
    std::atomic<double> quat[4];
    std::atomic<double> vel[3];
    std::atomic<double> angular_vel[3];
    std::unique_ptr<std::thread> hovering_thread;
    std::atomic<bool> hovering;

    void self_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->position[0].store(msg->pose.pose.position.x);
        this->position[1].store(msg->pose.pose.position.y);
        this->position[2].store(msg->pose.pose.position.z);

        this->sim_time.store(msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec);

        this->quat[0].store(msg->pose.pose.orientation.w);
        this->quat[1].store(msg->pose.pose.orientation.x);
        this->quat[2].store(msg->pose.pose.orientation.y);
        this->quat[3].store(msg->pose.pose.orientation.z);

        this->vel[0].store(msg->twist.twist.linear.x);
        this->vel[1].store(msg->twist.twist.linear.y);
        this->vel[2].store(msg->twist.twist.linear.z);

        this->angular_vel[0].store(msg->twist.twist.angular.x);
        this->angular_vel[1].store(msg->twist.twist.angular.y);
        this->angular_vel[2].store(msg->twist.twist.angular.z);
    }

    void tracking_target_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        this->tracking_target_position[0].store(msg->pose.pose.position.x);
        this->tracking_target_position[1].store(msg->pose.pose.position.y);
        this->tracking_target_position[2].store(msg->pose.pose.position.z);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        for(const auto &param : parameters)
        {
            if(param.get_name() == "pos_p") this->pos_p = param.as_double();
            else if(param.get_name() == "pos_i") this->pos_i = param.as_double();
            else if(param.get_name() == "pos_d") this->pos_d = param.as_double();
            else if(param.get_name() == "rot_p") this->rot_p = param.as_double_array();
            else if(param.get_name() == "rot_d") this->rot_d = param.as_double_array();
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const MoveQuad::Goal> goal)
    {
        if(goal->position.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal position must have 3 elements");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if(goal->is_relative_to_self)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal position request at x=~%g, y=~%g, z=~%g", goal->position[0], goal->position[1], goal->position[2]);
        }
        else if(goal->is_relative_to_target)
        {
            if(goal->target_odom_topic == "")
            {
                RCLCPP_ERROR(this->get_logger(), "No target is provided");
                return rclcpp_action::GoalResponse::REJECT;
            }
            tracking_target_odom_sub = create_subscription<nav_msgs::msg::Odometry>(
                goal->target_odom_topic,
                10,
                std::bind(&PositionControlServer::tracking_target_odom_callback, this, std::placeholders::_1)
            );
            RCLCPP_INFO(this->get_logger(), "Received goal position request at x=^%g, y=^%g, z=^%g", goal->position[0], goal->position[1], goal->position[2]);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received goal position request at x=%g, y=%g, z=%g", goal->position[0], goal->position[1], goal->position[2]);
        }
        if(goal->turn_off_motors_after_reach)
        {
            RCLCPP_INFO(this->get_logger(), "Force will be stop applying after reaching the goal");
        }
        (void)uuid;     // ignore the argument
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        using namespace std::placeholders;
        std::thread{std::bind(&PositionControlServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<MoveQuad::Feedback>();
        auto &current_pos = feedback->current_position;
        current_pos.resize(3);
        auto result = std::make_shared<MoveQuad::Result>();
        std::vector<double> goal_pos = goal->position;
        double time_limit = goal->time_limit > 0 ? goal->time_limit : 20.0;
        double error_tolerance = goal->error_tolerance > 0 ? goal->error_tolerance : 0.1;
        bool succeed = false;
        double start_time;
        double sim_time_;   // temp variable for storing sim_time
        hovering.store(false);
        if(hovering_thread && hovering_thread->joinable()) hovering_thread->join();

        if(goal->is_relative_to_self)
        {
            for(int i = 0; i < 3; i++) goal_pos[i] += this->position[i].load();
        }
        else if(goal->is_relative_to_target)
        {
            for(int i = 0; i < 3; i++) goal_pos[i] += this->tracking_target_position[i].load();
        }
        start_time = sim_time_ = this->sim_time.load();

        double fre = 10.0;
        rclcpp::Rate rate(fre);
        double prev_time = sim_time_ - 1.0 / fre;
        std::vector<double> error_pos(3);
        std::vector<double> prev_error(3);
        std::vector<IntegratedError> integrated_error(3);

        for(int i = 0; i < 3; i++) prev_error[i] = goal_pos[i] - this->position[i].load();

        while(rclcpp::ok() && sim_time_ < start_time + time_limit)
        {
            for(int i = 0; i < 3; i++) current_pos[i] = this->position[i].load();
            if(goal->is_relative_to_target)
            {
                // tracking the target in real time
                for(int i = 0; i < 3; i++) goal_pos[i] = goal->position[i] + this->tracking_target_position[i].load();
            }

            // check if goal is canceled
            if(goal_handle->is_canceling())
            {
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                result->final_position = current_pos;
                goal_handle->canceled(result);
                hovering.store(true);
                hovering_thread = std::make_unique<std::thread>(&PositionControlServer::hover, this);
                return;
            }

            std::vector<double> force(3);
            std::vector<double> torque(3);

            goal_handle->publish_feedback(feedback);
            for(int i = 0; i < 3; i++) error_pos[i] = goal_pos[i] - current_pos[i];
            if(error_pos[0] * error_pos[0] + error_pos[1] * error_pos[1] + error_pos[2] * error_pos[2] < error_tolerance * error_tolerance)
            {
                succeed = true;
                break;
            }

            translation_pid(error_pos, prev_error, integrated_error, sim_time, prev_time, force);

            std::vector<double> goal_quat;
            double axis[2];     // rotation axis
            double theta;       // rotation angle
            double force_proj = sqrt(force[0] * force[0] + force[1] * force[1]);
            if(force_proj < 0.001 * force[2])
            {
                goal_quat = {1, 0, 0, 0};
            }
            else
            {
                axis[0] = -force[1] / force_proj;
                axis[1] = force[0] / force_proj;
                theta = acos(force[2] / sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]));
                goal_quat = {cos(theta / 2), axis[0] * sin(theta / 2), axis[1] * sin(theta / 2), 0};
            }
            std::vector<double> current_quat(4);
            std::vector<double> angular_vel_(3);

            for(int i = 0; i < 4; i++) current_quat[i] = this->quat[i].load();
            for(int i = 0; i < 3; i++) angular_vel_[i] = this->angular_vel[i].load();

            rotation_pd(goal_quat, current_quat, angular_vel_, torque);
            geometry_msgs::msg::Wrench msg;
            msg.force.x = force[0];
            msg.force.y = force[1];
            msg.force.z = force[2];
            msg.torque.x = torque[0];
            msg.torque.y = torque[1];
            msg.torque.z = torque[2];
            force_pub->publish(msg);
            // RCLCPP_INFO(this->get_logger(), "Force: %g, %g, %g, Torque: %g, %g, %g", force[0], force[1], force[2], torque[0], torque[1], torque[2]);

            rate.sleep();
            sim_time_ = this->sim_time.load();
        }
        result->final_position = current_pos;
        result->final_error = error_pos;
        if(succeed)
        {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded, total time: %g", sim_time_ - start_time);
            goal_handle->succeed(result);
            if(goal->turn_off_motors_after_reach)
            {
                hovering.store(false);
                geometry_msgs::msg::Wrench msg;
                msg.force.x = 0;
                msg.force.y = 0;
                msg.force.z = 0;
                msg.torque.x = 0;
                msg.torque.y = 0;
                msg.torque.z = 0;
                force_pub->publish(msg);
            }
            else hovering.store(true);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal aborted");
            goal_handle->abort(result);
            hovering.store(true);
        }
        hovering_thread = std::make_unique<std::thread>(&PositionControlServer::hover, this);
        if(goal->is_relative_to_target) tracking_target_odom_sub.reset();
    }

    void translation_pid(const std::vector<double> &error,
        std::vector<double> &prev_error,
        std::vector<IntegratedError> &integrated_error,
        const double time,
        double &prev_time,
        std::vector<double> &force)
    {
        // PID control
        // force = kp * error + kd * (error - prev_error) / dt + ki * integrated_error + G
        double dt = time - prev_time;
        for(int i = 0; i < 3; i++)
        {
            force[i] = pos_p * error[i] + pos_d * (error[i] - prev_error[i]) / dt + pos_i * integrated_error[i].getvalue();
            integrated_error[i].push(error[i] * dt);
            prev_error[i] = error[i];
        }
        force[2] += G;
        prev_time = time;
    }

    void rotation_pd(const std::vector<double> &goal_quat,
        const std::vector<double> &current_quat,
        const std::vector<double> &angular_vel,
        std::vector<double> &torque)
    {
        // PD control
        // torque = kp * (goal_quat / current_quat) - kd * angular_vel
        std::vector<double> error_quat_imag(3);     // imaginary part of the error quaternion
        // error_quat = goal_quat / current_quat
        error_quat_imag[0] = -goal_quat[0] * current_quat[1] + goal_quat[1] * current_quat[0] - goal_quat[2] * current_quat[3] + goal_quat[3] * current_quat[2];
        error_quat_imag[1] = -goal_quat[0] * current_quat[2] + goal_quat[1] * current_quat[3] + goal_quat[2] * current_quat[0] - goal_quat[3] * current_quat[1];
        error_quat_imag[2] = -goal_quat[0] * current_quat[3] - goal_quat[1] * current_quat[2] + goal_quat[2] * current_quat[1] + goal_quat[3] * current_quat[0];
        for(int i = 0; i < 3; i++)
        {
            torque[i] = rot_p[i] * error_quat_imag[i] - rot_d[i] * angular_vel[i];
        }
    }

    void hover()
    {
        if(!hovering.load()) return;
        RCLCPP_INFO(this->get_logger(), "start hovering");
        double fre = 5.0;
        rclcpp::Rate rate(fre);
        std::vector<double> force(3);
        std::vector<double> torque(3);
        std::vector<double> current_vel(3);
        std::vector<double> prev_vel(3);
        double sim_time_ = this->sim_time.load();
        double prev_sim_time = sim_time_ - 1.0 / fre;

        for(int i = 0; i < 3; i++) prev_vel[i] = this->vel[i].load();
        while(rclcpp::ok() && hovering.load())
        {
            sim_time_ = this->sim_time.load();
            for(int i = 0; i < 3; i++) current_vel[i] = this->vel[i].load();
            // PD control
            // force = kp * (0 - current_vel) - kd * (current_vel - prev_vel) / dt + G
            double kp[3] = {1.0, 1.0, 1.0};
            double kd[3] = {0.1, 0.1, 0.1};
            double dt = sim_time_ - prev_sim_time;
            force[0] = kp[0] * (0 - current_vel[0]) - kd[0] * (current_vel[0] - prev_vel[0]) / dt;
            force[1] = kp[1] * (0 - current_vel[1]) - kd[1] * (current_vel[1] - prev_vel[1]) / dt;
            force[2] = kp[2] * (0 - current_vel[2]) - kd[2] * (current_vel[2] - prev_vel[2]) / dt + G;

            std::vector<double> current_quat(4);
            std::vector<double> angular_vel_(3);

            for(int i = 0; i < 4; i++) current_quat[i] = this->quat[i].load();
            for(int i = 0; i < 3; i++) angular_vel_[i] = this->angular_vel[i].load();
            rotation_pd({1,0,0,0}, current_quat, angular_vel_, torque);
            geometry_msgs::msg::Wrench msg;
            msg.force.x = force[0];
            msg.force.y = force[1];
            msg.force.z = force[2];
            msg.torque.x = torque[0];
            msg.torque.y = torque[1];
            msg.torque.z = torque[2];
            force_pub->publish(msg);

            prev_sim_time = sim_time_;
            for(int i = 0; i < 3; i++) prev_vel[i] = current_vel[i];
            rate.sleep();
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PositionControlServer>("position_control_server");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}