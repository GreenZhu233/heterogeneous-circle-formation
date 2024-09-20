#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "formation_interfaces/action/move_quad.hpp"
#include "quadrotor_control/visibility_control.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <thread>
#include <vector>
#include <mutex>
#include <cmath>

class PositionControlServer: public rclcpp::Node
{
public:
    using MoveQuad = formation_interfaces::action::MoveQuad;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveQuad>;

    explicit PositionControlServer(std::string name, const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node(name, options)
    {
        this->declare_parameter("gravity", 1.316*9.8);
        this->declare_parameter("odom_topic", "/odom");
        this->declare_parameter("force_topic", "/gazebo_ros_force");
        this->declare_parameter("action_service_name", "/position_control");
        this->gravity = this->get_parameter("gravity").as_double();
        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string force_topic = this->get_parameter("force_topic").as_string();
        std::string action_service_name = this->get_parameter("action_service_name").as_string();
        using namespace std::placeholders;
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
        this->odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            10,
            std::bind(&PositionControlServer::odom_callback, this, _1),
            sub_options
        );

        this->force_pub = this->create_publisher<geometry_msgs::msg::Twist>(
            force_topic,
            10
        );
    }

private:
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr force_pub;
    rclcpp_action::Server<MoveQuad>::SharedPtr action_server;
    double gravity;
    double position[3]; std::mutex position_mutex;
    double sim_time; std::mutex sim_time_mutex;
    double quat[4]; std::mutex quat_mutex;
    double vel[3]; std::mutex vel_mutex;
    double angular_vel[3]; std::mutex angular_vel_mutex;
    std::thread hovering_thread{std::bind(&PositionControlServer::hover, this)};
    bool hovering = false;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(this->position_mutex);
            this->position[0] = msg->pose.pose.position.x;
            this->position[1] = msg->pose.pose.position.y;
            this->position[2] = msg->pose.pose.position.z;
        }
        {
            std::lock_guard<std::mutex> lock(this->sim_time_mutex);
            this->sim_time = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
        }
        {
            std::lock_guard<std::mutex> lock(this->quat_mutex);
            this->quat[0] = msg->pose.pose.orientation.w;
            this->quat[1] = msg->pose.pose.orientation.x;
            this->quat[2] = msg->pose.pose.orientation.y;
            this->quat[3] = msg->pose.pose.orientation.z;
        }
        {
            std::lock_guard<std::mutex> lock(this->vel_mutex);
            this->vel[0] = msg->twist.twist.linear.x;
            this->vel[1] = msg->twist.twist.linear.y;
            this->vel[2] = msg->twist.twist.linear.z;
        }
        {
            std::lock_guard<std::mutex> lock(this->angular_vel_mutex);
            this->angular_vel[0] = msg->twist.twist.angular.x;
            this->angular_vel[1] = msg->twist.twist.angular.y;
            this->angular_vel[2] = msg->twist.twist.angular.z;
        }
    }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const MoveQuad::Goal> goal)
    {
        if(goal->position.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal position must have 3 elements");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "Received goal position request at x=%g, y=%g, z=%g", goal->position[0], goal->position[1], goal->position[2]);
        if(goal->turn_off_motors_after_reaching)
        {
            RCLCPP_INFO(this->get_logger(), "Stop applying force after reaching the goal");
        }
        (void)uuid;     // ignore the argument
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        hovering = true;
        hovering_thread.detach();
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
        double time_limit = goal->time_limit > 0 ? goal->time_limit : 10.0;
        double error_tolerance = goal->error_tolerance > 0 ? goal->error_tolerance : 0.1;
        bool succeed = false;
        double start_time;
        double sim_time_;
        hovering = false;
        if(hovering_thread.joinable()) hovering_thread.join();

        if(goal->relative)
        {
            std::lock_guard<std::mutex> lock(this->position_mutex);
            for(int i = 0; i < 3; i++) goal_pos[i] += this->position[i];
        }
        {
            std::lock_guard<std::mutex> lock(this->sim_time_mutex);
            start_time = sim_time_ = this->sim_time;
        }

        double fre = 30.0;
        rclcpp::Rate rate(fre);
        double prev_time = 1.0 / fre;
        std::vector<double> error_pos(3);
        std::vector<double> prev_error(3);
        std::vector<double> sum_error{0, 0, 0};
        {
            std::lock_guard<std::mutex> lock(this->position_mutex);
            for(int i = 0; i < 3; i++) prev_error[i] = goal_pos[i] - this->position[i];
        }
        while(rclcpp::ok() && sim_time_ < start_time + time_limit)
        {
            std::vector<double> force(3);
            std::vector<double> torque(3);
            {
                std::lock_guard<std::mutex> lock(this->position_mutex);
                for(int i = 0; i < 3; i++) current_pos[i] = this->position[i];
            }
            goal_handle->publish_feedback(feedback);
            for(int i = 0; i < 3; i++) error_pos[i] = goal_pos[i] - current_pos[i];
            if(error_pos[0] * error_pos[0] + error_pos[1] * error_pos[1] + error_pos[2] * error_pos[2] < error_tolerance * error_tolerance)
            {
                succeed = true;
                break;
            }
            translation_pid(error_pos, prev_error, sum_error, sim_time, start_time, prev_time, force);

            double axis[2];     // rotation axis
            double theta;       // rotation angle
            axis[0] = force[1] / sqrt(force[0] * force[0] + force[1] * force[1]);
            axis[1] = -force[0] / sqrt(force[0] * force[0] + force[1] * force[1]);
            theta = acos(force[2] / sqrt(force[0] * force[0] + force[1] * force[1] + force[2] * force[2]));
            std::vector<double> goal_quat{cos(theta / 2), axis[0] * sin(theta / 2), axis[1] * sin(theta / 2), 0};
            std::vector<double> current_quad(4);
            std::vector<double> angular_vel_(3);
            {
                std::lock_guard<std::mutex> lock(this->quat_mutex);
                for(int i = 0; i < 4; i++) current_quad[i] = this->quat[i];
            }
            {
                std::lock_guard<std::mutex> lock(this->angular_vel_mutex);
                for(int i = 0; i < 3; i++) angular_vel_[i] = this->angular_vel[i];
            }
            rotation_pd(goal_quat, current_quad, angular_vel_, torque);
            geometry_msgs::msg::Twist msg;
            msg.linear.x = force[0];
            msg.linear.y = force[1];
            msg.linear.z = force[2];
            msg.angular.x = torque[0];
            msg.angular.y = torque[1];
            msg.angular.z = torque[2];
            force_pub->publish(msg);

            rate.sleep();
            {
                std::lock_guard<std::mutex> lock(this->sim_time_mutex);
                sim_time_ = this->sim_time;
            }
        }
        result->final_position = current_pos;
        if(succeed)
        {
            goal_handle->succeed(result);
            if(goal->turn_off_motors_after_reaching)
            {
                hovering = false;
                geometry_msgs::msg::Twist msg;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                msg.angular.y = 0;
                msg.angular.z = 0;
                force_pub->publish(msg);
            }
            else hovering = true;
        }
        else
        {
            goal_handle->abort(result);
            hovering = true;
        }
        hovering_thread.detach();
    }

    void translation_pid(const std::vector<double> &error,
        std::vector<double> &prev_error,
        std::vector<double> &sum_error,
        const double time,
        const double start_time,
        double &prev_time,
        std::vector<double> &force)
    {
        // PID control
        // force = kp * error + kd * (error - prev_error) / dt + ki * sum_error / T
        double kp = 1.0, ki = 0.1, kd = 0.1;
        double dt = time - prev_time;
        double T = time - start_time;
        for(int i = 0; i < 3; i++)
        {
            force[i] = kp * error[i] + kd * (error[i] - prev_error[i]) / dt + ki * sum_error[i] / T;
            sum_error[i] += error[i];
            prev_error[i] = error[i];
        }
        force[2] += gravity;
        prev_time = time;
    }

    void rotation_pd(const std::vector<double> &goal_quat,
        const std::vector<double> &current_quad,
        const std::vector<double> &angular_vel,
        std::vector<double> &torque)
    {
        // PD control
        // torque = kp * (goal_quat / current_quad) - kd * angular_vel
        const std::vector<double> kp{0.02, 0.02, 0.04};
        const std::vector<double> kd{0.1, 0.1, 0.2};
        std::vector<double> error_quat_imag(3);     // imaginary part of the error quaternion
        error_quat_imag[0] = -goal_quat[0] * current_quad[1] + goal_quat[1] * current_quad[0] - goal_quat[2] * current_quad[3] + goal_quat[3] * current_quad[2];
        error_quat_imag[1] = -goal_quat[0] * current_quad[2] + goal_quat[1] * current_quad[3] + goal_quat[2] * current_quad[0] - goal_quat[3] * current_quad[1];
        error_quat_imag[2] = -goal_quat[0] * current_quad[3] - goal_quat[1] * current_quad[2] + goal_quat[2] * current_quad[1] + goal_quat[3] * current_quad[0];
        for(int i = 0; i < 3; i++)
        {
            torque[i] = kp[i] * error_quat_imag[i] - kd[i] * angular_vel[i];
        }
    }

    void hover()
    {
        if(!hovering) return;
        double fre = 5.0;
        rclcpp::Rate rate(fre);
        std::vector<double> force(3);
        std::vector<double> torque(3);
        std::vector<double> present_vel(3);
        std::vector<double> prev_vel(3);
        double sim_time_;
        {
            std::lock_guard<std::mutex> lock(this->sim_time_mutex);
            sim_time_ = this->sim_time;
        }
        double prev_sim_time = sim_time_ - 1.0 / fre;
        {
            std::lock_guard<std::mutex> lock(this->vel_mutex);
            for(int i = 0; i < 3; i++) prev_vel[i] = this->vel[i];
        }
        while(rclcpp::ok() && hovering)
        {
            {
                std::lock_guard<std::mutex> lock(this->sim_time_mutex);
                sim_time_ = this->sim_time;
            }
            {
                std::lock_guard<std::mutex> lock(this->vel_mutex);
                for(int i = 0; i < 3; i++) present_vel[i] = this->vel[i];
            }
            // PD control
            // force = kp * (0 - present_vel) - kd * (present_vel - prev_vel) / dt + gravity
            double kp[3] = {0.1, 0.1, 0.1};
            double kd[3] = {0.1, 0.1, 0.1};
            double dt = sim_time_ - prev_sim_time;
            force[0] = kp[0] * (0 - present_vel[0]) - kd[0] * (present_vel[0] - prev_vel[0]) / dt;
            force[1] = kp[1] * (0 - present_vel[1]) - kd[1] * (present_vel[1] - prev_vel[1]) / dt;
            force[2] = kp[2] * (0 - present_vel[2]) - kd[2] * (present_vel[2] - prev_vel[2]) / dt + gravity;

            std::vector<double> current_quad(4);
            std::vector<double> angular_vel_(3);
            {
                std::lock_guard<std::mutex> lock(this->quat_mutex);
                for(int i = 0; i < 4; i++) current_quad[i] = this->quat[i];
            }
            {
                std::lock_guard<std::mutex> lock(this->angular_vel_mutex);
                for(int i = 0; i < 3; i++) angular_vel_[i] = this->angular_vel[i];
            }
            rotation_pd({1,0,0,0}, current_quad, angular_vel_, torque);
            geometry_msgs::msg::Twist msg;
            msg.linear.x = force[0];
            msg.linear.y = force[1];
            msg.linear.z = force[2];
            msg.angular.x = torque[0];
            msg.angular.y = torque[1];
            msg.angular.z = torque[2];
            force_pub->publish(msg);

            prev_sim_time = sim_time_;
            for(int i = 0; i < 3; i++) prev_vel[i] = present_vel[i];
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