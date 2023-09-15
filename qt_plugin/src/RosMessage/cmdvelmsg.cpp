#include "rosmessage/cmdvelmsg.hpp"

CmdVel::CmdVel() : Node("cmdvel"), count_(0)
{

        

        cmd_vel_A = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&CmdVel::cmd_vel_callback, this, std::placeholders::_1));
        cmd_vel_B = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_B", 10, std::bind(&CmdVel::cmd_vel_callback_B, this, std::placeholders::_1));
        cmd_vel_V = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_V", 10, std::bind(&CmdVel::cmd_vel_callback_V, this, std::placeholders::_1));
}

void CmdVel::cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
        QString linear_x=QString::number(msg->linear.x,'f',2);
        QString angular_z=QString::number(msg->linear.x,'f',2);
        // ui_->actual_lineal_speed->setText(QString::number(msg->linear.x,'f',2));
        // ui_->actual_angual_speed->setText(QString::number(msg->angular.z,'f',2));
}
void CmdVel::cmd_vel_callback_B(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
        QString linear_x=QString::number(msg->linear.x,'f',2);
        QString angular_z=QString::number(msg->linear.x,'f',2);
        // ui_->actual_lineal_speed_B->setText(QString::number(msg->linear.x,'f',2));
        // ui_->actual_angual_speed_B->setText(QString::number(msg->angular.z,'f',2));
}
void CmdVel::cmd_vel_callback_V(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
        QString linear_x=QString::number(msg->linear.x,'f',2);
        QString angular_z=QString::number(msg->linear.x,'f',2);
        // ui_->actual_lineal_speed_V->setText(QString::number(msg->linear.x,'f',2));
        // ui_->actual_angual_speed_V->setText(QString::number(msg->angular.z,'f',2));
}