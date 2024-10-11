#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/accel.hpp"

#include <string>
#include <vector>
#include <atomic>
#include <Eigen/Dense>

using Odometry = nav_msgs::msg::Odometry;
using Accel = geometry_msgs::msg::Accel;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

double tolerance = 0.1;

class PIDRegulator
{
private:
    double Kp;
    double Ki;
    double Kd;
    double sat;
    Eigen::Vector3d integral;
    Eigen::Vector3d prev_error;
    double prev_t;

public:
    PIDRegulator(double Kp, double Ki, double Kd, double sat)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->sat = sat;
        integral = Eigen::Vector3d::Zero();
        prev_error = Eigen::Vector3d::Zero();
        prev_t = -1.0;
    }

    Eigen::Vector3d regulate(Eigen::Vector3d error, double t)
    {
        if(prev_t < 0.0)
        {
            prev_t = t;
            return Eigen::Vector3d::Zero();
        }
        double dt = t - prev_t;
        Eigen::Vector3d derivative = Eigen::Vector3d::Zero();
        if(dt > 0.0)
        {
            derivative = (error - prev_error) / dt;
            integral += error * dt;
        }
        Eigen::Vector3d output = Kp * error + Ki * integral + Kd * derivative;
        if(output.norm() > sat)
        {
            output = output / output.norm() * sat;
        }
        prev_error = error;
        prev_t = t;
        return output;
    }
};

class Rexrov
{
public:
    int id;
    std::string ns;
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> z;
    std::atomic<double> ori[4];     // orientaion quaternion (w,x,y,z)
    std::atomic<double> vx;         // linear velocity
    std::atomic<double> vy;
    std::atomic<double> vz;
    std::atomic<double> wx;         // angular velocity
    std::atomic<double> wy;
    std::atomic<double> wz;
    double phi;

    Rexrov(int id, std::string ns)
    {
        this->id = id;
        this->ns = ns;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x.store(msg->pose.pose.position.x);
        y.store(msg->pose.pose.position.y);
        z.store(msg->pose.pose.position.z);
        ori[0].store(msg->pose.pose.orientation.w);
        ori[1].store(msg->pose.pose.orientation.x);
        ori[2].store(msg->pose.pose.orientation.y);
        ori[3].store(msg->pose.pose.orientation.z);
        vx.store(msg->twist.twist.linear.x);
        vy.store(msg->twist.twist.linear.y);
        vz.store(msg->twist.twist.linear.z);
        wx.store(msg->twist.twist.angular.x);
        wy.store(msg->twist.twist.angular.y);
        wz.store(msg->twist.twist.angular.z);
    }
};

bool cmp(std::shared_ptr<Rexrov> prexrov1, std::shared_ptr<Rexrov> prexrov2, double center[])
{
    if(prexrov1->phi - prexrov2->phi < -tolerance) return true;
    if(prexrov1->phi - prexrov2->phi > tolerance) return false;
    double r1 = fabs(prexrov1->x.load() - center[0]) + fabs(prexrov1->y.load() - center[1]);
    double r2 = fabs(prexrov2->x.load() - center[0]) + fabs(prexrov2->y.load() - center[1]);
    if(r1 < r2) return true;
    return false;
}

class RexrovSteeringNode : public rclcpp_lifecycle::LifecycleNode
{
private:
    std::string name;
    int num;
    std::vector<std::shared_ptr<Rexrov>> rexrovs;
    std::vector<rclcpp::Publisher<Accel>::SharedPtr> pubs;
    std::vector<rclcpp::Subscription<Odometry>::SharedPtr> subs;
    double center[3];
    double radius;
    double Lambda;
    double Gamma;
    double C1;
    double C2;
    double rot_p;
    double rot_d;
    double linear_p;
    double linear_i;
    double linear_d;
    std::unique_ptr<PIDRegulator> pid_regulator;
    rclcpp::TimerBase::SharedPtr timer;
    double **p_bar;
    double *d;
    double *theta;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_event_subscriber_;

public:
    RexrovSteeringNode(std::string name) : rclcpp_lifecycle::LifecycleNode(name)
    {
        this->name = name;
        RCLCPP_INFO(get_logger(), "%s initialized", name.c_str());
        declare_parameter<int>("num", 5);
        declare_parameter<double>("center_x", -7.0);
        declare_parameter<double>("center_y", 0.0);
        declare_parameter<double>("center_z", -0.7);
        declare_parameter<double>("radius", 5.0);
        declare_parameter<double>("Lambda", 1.2);
        declare_parameter<double>("Gamma", 0.01);
        declare_parameter<double>("C1", 0.4);
        declare_parameter<double>("C2", 1.0);
        declare_parameter<double>("rot_p", 1.0);
        declare_parameter<double>("rot_d", 0.5);
        declare_parameter<double>("linear_p", 1.0);
        declare_parameter<double>("linear_d", 0.5);
        declare_parameter<double>("linear_i", 0.001);
        declare_parameter<double>("linear_sat", 1.0);
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "%s configured", name.c_str());
        num = get_parameter("num").as_int();
        center[0] = get_parameter("center_x").as_double();
        center[1] = get_parameter("center_y").as_double();
        center[2] = get_parameter("center_z").as_double();
        radius = get_parameter("radius").as_double();
        Lambda = get_parameter("Lambda").as_double();
        Gamma = get_parameter("Gamma").as_double();
        C1 = get_parameter("C1").as_double();
        C2 = get_parameter("C2").as_double();
        rot_p = get_parameter("rot_p").as_double();
        rot_d = get_parameter("rot_d").as_double();
        linear_p = get_parameter("linear_p").as_double();
        linear_i = get_parameter("linear_i").as_double();
        linear_d = get_parameter("linear_d").as_double();
        parameter_event_subscriber_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &parameters) -> rcl_interfaces::msg::SetParametersResult
            {
                parameterCallback(parameters);
                return rcl_interfaces::msg::SetParametersResult();
            }
        );

        rexrovs.resize(num);
        subs.resize(num);
        pubs.resize(num);
        for(int i = 0; i < num; i++)
        {
            rexrovs[i] = std::make_shared<Rexrov>(i, std::string("rexrov_") + std::to_string(i));
            pubs[i] = create_publisher<Accel>(std::string("/") + rexrovs[i]->ns + std::string("/cmd_accel"), 10);
            subs[i] = create_subscription<Odometry>(
                std::string("/") + rexrovs[i]->ns + std::string("/pose_gt"),
                10,
                std::bind(&Rexrov::odom_callback, rexrovs[i], std::placeholders::_1)
            );
        }
        p_bar = new double*[num];
        for(int i = 0; i < num; ++i) p_bar[i] = new double[2];
        d = new double[num];
        for(int i = 0; i < num; ++i) d[i] = 2 * M_PI / num;
        theta = new double[num];

        pid_regulator = std::make_unique<PIDRegulator>(linear_p, linear_i, linear_d, 2.0);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "%s activated", name.c_str());
        counterclockwise_sort();
        timer = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RexrovSteeringNode::circle_formation, this)
        );
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "%s deactivated", name.c_str());
        timer.reset();
        Accel cmd;
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
        for(int i = 0; i < num; i++)
        {
            pubs[i]->publish(cmd);
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "%s cleaned up", name.c_str());
        timer.reset();
        rexrovs.clear();
        pubs.clear();
        subs.clear();
        if(p_bar)
            for(int i = 0; i < num; ++i) delete[] p_bar[i];
        delete[] p_bar; p_bar = nullptr;
        delete[] d; d = nullptr;
        delete[] theta; theta = nullptr;
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(get_logger(), "%s shut down", name.c_str());
        timer.reset();
        rexrovs.clear();
        pubs.clear();
        subs.clear();
        if(p_bar)
            for(int i = 0; i < num; ++i) delete[] p_bar[i];
        delete[] p_bar; p_bar = nullptr;
        delete[] d; d = nullptr;
        delete[] theta; theta = nullptr;
        return CallbackReturn::SUCCESS;
    }

private:
    void counterclockwise_sort()
    {
        for(auto prexrov: rexrovs)
        {
            prexrov->phi = atan2(prexrov->y.load() - center[1], prexrov->x.load() - center[0]);
        }
        std::sort(rexrovs.begin(), rexrovs.end(), std::bind(cmp, std::placeholders::_1, std::placeholders::_2, center));
    }

    void circle_formation()
    {
        for(int i = 0; i < num; i++)
        {
            p_bar[i][0] = rexrovs[i]->x.load() - center[0];
            p_bar[i][1] = rexrovs[i]->y.load() - center[1];
            theta[i] = atan2(p_bar[i][1], p_bar[i][0]);
        }
        for(int i = 0; i < num; i++)
        {
            int i_plus = (i + 1) % num;
            int i_minus = (i - 1 + num) % num;
            double alpha = theta[i_plus] - theta[i];
            if(alpha < -tolerance) alpha += 2 * M_PI;
            double alpha_minus = theta[i] - theta[i_minus];
            if(alpha_minus < -tolerance) alpha_minus += 2 * M_PI;
            double l = radius * radius - (p_bar[i][0] * p_bar[i][0] + p_bar[i][1] * p_bar[i][1]);
            double f = C1 + C2 / (2 * M_PI) * (d[i_minus] * alpha - d[i] * alpha_minus) / (d[i_minus] + d[i]);
            double vx = Lambda * f * (Gamma * l * p_bar[i][0] - p_bar[i][1]);
            double vy = Lambda * f * (p_bar[i][0] + Gamma * l * p_bar[i][1]);
            double vz = 1.0 * (center[2] - rexrovs[i]->z.load());
            Accel cmd;
            // coordinate transformation
            Eigen::Quaterniond q(
                rexrovs[i]->ori[0].load(),
                rexrovs[i]->ori[1].load(),
                rexrovs[i]->ori[2].load(),
                rexrovs[i]->ori[3].load()
            );
            Eigen::Matrix3d R_wb = q.normalized().toRotationMatrix();
            Eigen::Matrix3d R_bw = R_wb.transpose();
            Eigen::Vector3d goal_v_world(vx, vy, vz);
            Eigen::Vector3d goal_v_body = R_bw * goal_v_world;
            Eigen::Vector3d current_v_body(
                rexrovs[i]->vx.load(),
                rexrovs[i]->vy.load(),
                rexrovs[i]->vz.load()
            );
            Eigen::Vector3d error_v_body = goal_v_body - current_v_body;
            Eigen::Vector3d accel_body = pid_regulator->regulate(error_v_body, get_clock()->now().seconds());
            cmd.linear.x = accel_body(0);
            cmd.linear.y = accel_body(1);
            cmd.linear.z = accel_body(2);

            // angular velocity
            double goal_yaw = atan2(vy, vx);
            Eigen::Quaterniond goal_q(
                cos(goal_yaw / 2),
                0,
                0,
                sin(goal_yaw / 2)
            );
            // Eigen::Quaterniond goal_q(1,0,0,0);
            Eigen::Quaterniond error_q = q.conjugate() * goal_q;
            if(error_q.w() < 0)
            {
                error_q.coeffs() = -error_q.coeffs();
            }
            Eigen::Vector3d angular_velocity_body(
                rexrovs[i]->wx.load(),
                rexrovs[i]->wy.load(),
                rexrovs[i]->wz.load()
            );
            Eigen::Vector3d angular_accel_body = rot_p * error_q.vec() - rot_d * angular_velocity_body;
            cmd.angular.x = angular_accel_body(0);
            cmd.angular.y = angular_accel_body(1);
            cmd.angular.z = angular_accel_body(2);

            // publish velocity command
            pubs[rexrovs[i]->id]->publish(cmd);
        }
    }

    void parameterCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        for(const auto &parameter : parameters)
        {
            if(parameter.get_name() == std::string("radius"))
            {
                radius = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("Lambda"))
            {
                Lambda = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("Gamma"))
            {
                Gamma = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("C1"))
            {
                C1 = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("C2"))
            {
                C2 = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("rot_p"))
            {
                rot_p = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("rot_d"))
            {
                rot_d = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("linear_p"))
            {
                linear_p = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("linear_i"))
            {
                linear_i = parameter.as_double();
            }
            else if(parameter.get_name() == std::string("linear_d"))
            {
                linear_d = parameter.as_double();
            }
            pid_regulator = std::make_unique<PIDRegulator>(linear_p, linear_i, linear_d, 2.0);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RexrovSteeringNode>("rexrov_steering_node");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}