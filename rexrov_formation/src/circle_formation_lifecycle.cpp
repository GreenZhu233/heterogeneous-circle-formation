#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <vector>
#include <atomic>
#include <Eigen/Dense>

using Odometry = nav_msgs::msg::Odometry;
using Twist = geometry_msgs::msg::Twist;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

double tolerance = 0.1;

class Rexrov
{
public:
    int id;
    std::string ns;
    std::atomic<double> x;
    std::atomic<double> y;
    std::atomic<double> z;
    std::atomic<double> ori[4];     // orientaion quaternion (w,x,y,z)
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
    std::vector<rclcpp::Publisher<Twist>::SharedPtr> pubs;
    std::vector<rclcpp::Subscription<Odometry>::SharedPtr> subs;
    double center[3];
    double radius;
    double Lambda;
    double Gamma;
    double C1;
    double C2;
    double rot_p;
    rclcpp::TimerBase::SharedPtr timer;
    double **p_bar;
    double *d;
    double *theta;

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

        rexrovs.resize(num);
        subs.resize(num);
        pubs.resize(num);
        for(int i = 0; i < num; i++)
        {
            rexrovs[i] = std::make_shared<Rexrov>(i, std::string("rexrov_") + std::to_string(i));
            pubs[i] = create_publisher<Twist>(std::string("/") + rexrovs[i]->ns + std::string("/cmd_vel"), 10);
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
        Twist cmd;
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
            Twist cmd;
            // coordinate transformation
            Eigen::Quaterniond q(
                rexrovs[i]->ori[0].load(),
                rexrovs[i]->ori[1].load(),
                rexrovs[i]->ori[2].load(),
                rexrovs[i]->ori[3].load()
            );
            Eigen::Matrix3d world_to_body = q.normalized().toRotationMatrix();
            Eigen::Matrix3d body_to_world = world_to_body.transpose();
            Eigen::Vector3d v_world(vx, vy, vz);
            Eigen::Vector3d v_body = body_to_world * v_world;
            cmd.linear.x = v_body(0);
            cmd.linear.y = v_body(1);
            cmd.linear.z = v_body(2);

            // angular velocity
            double goal_yaw = atan2(vy, vx);
            Eigen::Quaterniond goal_q(
                cos(goal_yaw / 2),
                0,
                0,
                sin(goal_yaw / 2)
            );
            Eigen::Quaterniond error_q = goal_q * q.inverse();
            if(error_q.w() < 0)
            {
                error_q.coeffs() = -error_q.coeffs();
            }
            Eigen::Vector3d angular_v = rot_p * body_to_world * error_q.vec();
            cmd.angular.x = angular_v(0);
            cmd.angular.y = angular_v(1);
            cmd.angular.z = angular_v(2);

            // publish velocity command
            pubs[rexrovs[i]->id]->publish(cmd);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RexrovSteeringNode>("rexrov_formation_lifecycle_node");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}