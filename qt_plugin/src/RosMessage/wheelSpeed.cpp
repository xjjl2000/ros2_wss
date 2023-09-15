#include "rosmessage/wheelSpeed.hpp"
#include "car_init.hpp"

WheelSpeed::WheelSpeed() : Node("wheelspeed"), count_(0)
{
        jointstate_V = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states_isaac", 10, std::bind(&WheelSpeed::jointState_callback, this, std::placeholders::_1));
        nmp_A = this->create_subscription<nav_msgs::msg::Odometry>("/odom_combined_A", 10, std::bind(&WheelSpeed::speedA, this, std::placeholders::_1));
        nmp_B = this->create_subscription<nav_msgs::msg::Odometry>("/odom_combined_B", 10, std::bind(&WheelSpeed::speedB, this, std::placeholders::_1));
}

void WheelSpeed::speedA(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
        wheel_npm A;

        A.fl = msg->twist.twist.linear.x - msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
        A.fr = msg->twist.twist.linear.x + msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
        A.rr = msg->twist.twist.linear.x + msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
        A.rl = msg->twist.twist.linear.x - msg->twist.twist.linear.z * (car_setting::Wheel_spacing + car_setting::Wheel_axlespacing);
        QString fl_Speed=QString::number(A.fl,'f',2);
        QString fr_Speed=QString::number(A.fr,'f',2);
        QString rl_Speed=QString::number(A.rl,'f',2);
        QString rr_Speed=QString::number(A.rr,'f',2);

        // ui_->fl_Speed->setText(QString::number(A.fl,'f',2));
        // ui_->fr_Speed->setText(QString::number(A.fr,'f',2));
        // ui_->rl_Speed->setText(QString::number(A.rl,'f',2));
        // ui_->rr_Speed->setText(QString::number(A.rr,'f',2));
}

void WheelSpeed::speedB(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
        wheel_npm B;
        float R = msg->twist.twist.linear.x / msg->twist.twist.linear.z;

        B.fl = msg->twist.twist.linear.x * cos(atan(car_setting::Wheel_axlespacing_B * R));
        B.fr = msg->twist.twist.linear.x * cos(atan(car_setting::Wheel_axlespacing_B * R));
        B.rr = msg->twist.twist.linear.x * (R - 0.5 * car_setting::Wheel_spacing_B) / R;
        B.rl = msg->twist.twist.linear.x * (R + 0.5 * car_setting::Wheel_spacing_B) / R;
        QString fl_Speed_B=QString::number(B.fl,'f',2);
        QString fr_Speed_B=QString::number(B.fr,'f',2);
        QString rl_Speed_B=QString::number(B.rl,'f',2);
        QString rr_Speed_B=QString::number(B.rr,'f',2);
        // ui_->fl_Speed_B->setText(QString::number(B.fl,'f',2));
        // ui_->fr_Speed_B->setText(QString::number(B.fr,'f',2));
        // ui_->rl_Speed_B->setText(QString::number(B.rl,'f',2));
        // ui_->rr_Speed_B->setText(QString::number(B.rr,'f',2));
}

void WheelSpeed::jointState_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{

        QString fl_Speed_V=QString::number(msg->velocity[0],'f',2);
        QString fr_Speed_V=QString::number(msg->velocity[1],'f',2);
        QString rl_Speed_V=QString::number(msg->velocity[2],'f',2);
        QString rr_Speed_V=QString::number(msg->velocity[3],'f',2);
        // ui_->fl_Speed_V->setText(QString::number(msg->velocity[0],'f',2));
        // ui_->rl_Speed_V->setText(QString::number(msg->velocity[1],'f',2));
        // ui_->rr_Speed_V->setText(QString::number(msg->velocity[2],'f',2));
        // ui_->fr_Speed_V->setText(QString::number(msg->velocity[3],'f',2));
}
