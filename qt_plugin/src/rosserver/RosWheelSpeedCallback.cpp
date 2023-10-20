#include "car_init.hpp"
#include "rclInclude.hpp"
#include "rosserver/RosCallbackInterface.hpp"
#include "rosserver/DelayHz.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class Ros4WheelDifferentialCallback : public RosCallbackInterface<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry::ConstSharedPtr>
{

        void callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) override
        {
                wheel_npm A;

                A.fl = msg->twist.twist.linear.x - msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
                A.fr = msg->twist.twist.linear.x + msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
                A.rr = msg->twist.twist.linear.x + msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
                A.rl = msg->twist.twist.linear.x - msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
                QString fl_Speed = QString::number(A.fl, 'f', 2);
                QString fr_Speed = QString::number(A.fr, 'f', 2);
                QString rl_Speed = QString::number(A.rl, 'f', 2);
                QString rr_Speed = QString::number(A.rr, 'f', 2);
        }

public:
        Ros4WheelDifferentialCallback(const std::string topic, rclcpp::Node *node) : RosCallbackInterface<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry::ConstSharedPtr>(topic, node){};
};

class Ros2WheelDifferentialCallback : public RosCallbackInterface<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry::ConstSharedPtr>
{

        void callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) override
        {
                wheel_npm B;
                float R = msg->twist.twist.linear.x / msg->twist.twist.linear.z;

                B.fl = msg->twist.twist.linear.x * cos(atan(car_setting::Wheel_axlespacing_B * R));
                B.fr = msg->twist.twist.linear.x * cos(atan(car_setting::Wheel_axlespacing_B * R));
                B.rr = msg->twist.twist.linear.x * (R - 0.5 * car_setting::Wheel_spacing_B) / R;
                B.rl = msg->twist.twist.linear.x * (R + 0.5 * car_setting::Wheel_spacing_B) / R;
                QString fl_Speed_B = QString::number(B.fl, 'f', 2);
                QString fr_Speed_B = QString::number(B.fr, 'f', 2);
                QString rl_Speed_B = QString::number(B.rl, 'f', 2);
                QString rr_Speed_B = QString::number(B.rr, 'f', 2);
        }

public:
        Ros2WheelDifferentialCallback(const std::string topic, rclcpp::Node *node) : RosCallbackInterface<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry::ConstSharedPtr>(topic, node){};
};

class RosjointStateCallback : public RosCallbackInterface<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState::ConstSharedPtr>
{

        void callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg) override
        {
                QString fl_Speed_V = QString::number(msg->velocity[0], 'f', 2);
                QString fr_Speed_V = QString::number(msg->velocity[1], 'f', 2);
                QString rl_Speed_V = QString::number(msg->velocity[2], 'f', 2);
                QString rr_Speed_V = QString::number(msg->velocity[3], 'f', 2);
        }

public:
        RosjointStateCallback(const std::string topic, rclcpp::Node *node) : RosCallbackInterface<sensor_msgs::msg::JointState, sensor_msgs::msg::JointState::ConstSharedPtr>(topic, node){};
};