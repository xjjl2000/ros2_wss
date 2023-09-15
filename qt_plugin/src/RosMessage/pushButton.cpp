#include "rosmessage/pushButton.hpp"
#include "car_init.hpp"

PushButton::PushButton() : Node("pushbutton"), count_(0)
{
    // remote_control
    remote_control_pub_A = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    remote_control_pub_B = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_B", 10);
    remote_control_pub_V = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_V", 10);
    par_V = this->create_publisher<std_msgs::msg::String>("controller_parm", 10);
}

void PushButton::up_button_clicked()
{
    // geometry_msgs::Twist cmdmsg;
    // cmdmsg.set_angular_x(0);
    // cmdmsg.set_angular_y(0);
    // cmdmsg.set_angular_z(0);
    // cmdmsg.set_linear_x(1* car_setting::RemoteForwardSpeed);
    // cmdmsg.set_linear_y(0);
    // cmdmsg.set_linear_z(2);
    // std::string ser_msg;
    // cmdmsg.SerializeToString(&ser_msg);
    // std::cout<<"1111"<<std::endl;
    // mqtt_cmd_pub.pub("mqtt_cmd_vel",ser_msg,0);
    // // auto remote_control_msg = geometry_msgs::msg::Twist();
    // // PushButton button;
    // // button.up_(remote_control_msg);
    // // remote_control_pub_A->publish(remote_control_msg);
    // QString text = "前进";
    // ui_->mode->setText(text);
}
void PushButton::down_button_clicked()
{

    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.down_(remote_control_msg);
    remote_control_pub_A->publish(remote_control_msg);
    QString text = "后退";
    // ui_->mode->setText(text);
}
void PushButton::left_button_clicked()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.left_(remote_control_msg);
    remote_control_pub_A->publish(remote_control_msg);
    QString text = "左转";
    // ui_->mode->setText(text);
}
void PushButton::right_button_clicked()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.right_(remote_control_msg);
    remote_control_pub_A->publish(remote_control_msg);
    QString text = "右转";
    // ui_->mode->setText(text);
}
void PushButton::stop_button_clicked()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.stop_(remote_control_msg);
    remote_control_pub_A->publish(remote_control_msg);
    QString text = "停止";
    // ui_->mode->setText(text);
}

void PushButton::up_button_clicked_B()
{

    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.up_B(remote_control_msg);
    remote_control_pub_B->publish(remote_control_msg);
    QString text = "前进";
    // ui_->mode_B->setText(text);
}
void PushButton::down_button_clicked_B()
{

    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.down_B(remote_control_msg);
    remote_control_pub_B->publish(remote_control_msg);
    QString text = "后退";
    // ui_->mode_B->setText(text);
}
void PushButton::left_button_clicked_B()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.left_B(remote_control_msg);
    remote_control_pub_B->publish(remote_control_msg);
    QString text = "左转";
    // ui_->mode_B->setText(text);
}
void PushButton::right_button_clicked_B()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.right_B(remote_control_msg);
    remote_control_pub_B->publish(remote_control_msg);
    QString text = "右转";
    // ui_->mode_B->setText(text);
}
void PushButton::stop_button_clicked_B()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.stop_B(remote_control_msg);
    remote_control_pub_B->publish(remote_control_msg);
    QString text = "停止";
    // ui_->mode_B->setText(text);
}

void PushButton::up_button_clicked_V()
{

    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.up_V(remote_control_msg);
    remote_control_pub_V->publish(remote_control_msg);
    QString text = "前进";
    // ui_->mode_V->setText(text);
}
void PushButton::down_button_clicked_V()
{

    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.left_V(remote_control_msg);
    remote_control_pub_V->publish(remote_control_msg);
    QString text = "后退";
    // ui_->mode_V->setText(text);
}
void PushButton::left_button_clicked_V()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.down_V(remote_control_msg);
    remote_control_pub_V->publish(remote_control_msg);
    QString text = "左转";
    // ui_->mode_V->setText(text);
}
void PushButton::right_button_clicked_V()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.right_V(remote_control_msg);
    remote_control_pub_V->publish(remote_control_msg);
    QString text = "右转";
    // ui_->mode_V->setText(text);
}
void PushButton::stop_button_clicked_V()
{
    auto remote_control_msg = geometry_msgs::msg::Twist();
    PushButton button;
    button.stop_V(remote_control_msg);
    remote_control_pub_V->publish(remote_control_msg);
    QString text = "停止";
    // ui_->mode_V->setText(text);
}

void PushButton::reset_button_clicked()
{
    car_setting::RemoteLeftSpeed = 0.15;
    car_setting::RemoteLeftAngularSpeed = 0.7;

    car_setting::RemoteLeftSpeed_B = 0.15;
    car_setting::RemoteLeftAngularSpeed_B = 0.7;

    car_setting::RemoteLeftSpeed_V = 0.15;
    car_setting::RemoteLeftAngularSpeed_V = 0.7;
}

void PushButton::controller_parm_button_clicked()
{
    // k=(ui_->k_->text()).toFloat();
    // lfc=(ui_->Lfc->text().toFloat());
    // max_speed=(ui_->max_speed->text().toFloat());
    // max_omega=(ui_->max_omege->text().toFloat());

    std::ostringstream oss;
    oss << "k = " << k << ", Lfc = " << lfc << ", max_speed = " << max_speed << ", max_omega = " << max_omega;

    std_msgs::msg::String par_msg;
    par_msg.data = oss.str();
    par_V->publish(par_msg);
}
// void PushButton::points_cleart_button_clicked()
// {
//     points.clear();
//     points_B.clear();
// }在前端处理

void PushButton::up_(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 1 * car_setting::RemoteForwardSpeed;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::down_(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = -1 * car_setting::RemoteForwardSpeed;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::left_(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 1 * car_setting::RemoteLeftAngularSpeed;
    msg.linear.x = 1 * car_setting::RemoteLeftSpeed;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::right_(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = -1 * car_setting::RemoteLeftAngularSpeed;
    msg.linear.x = 1 * car_setting::RemoteLeftSpeed;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::stop_(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
}

void PushButton::up_B(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 1 * car_setting::RemoteForwardSpeed_B;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::down_B(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = -1 * car_setting::RemoteForwardSpeed_B;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::left_B(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 1 * car_setting::RemoteLeftAngularSpeed_B;
    msg.linear.x = 1 * car_setting::RemoteLeftSpeed_B;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::right_B(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = -1 * car_setting::RemoteLeftAngularSpeed_B;
    msg.linear.x = 1 * car_setting::RemoteLeftSpeed_B;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::stop_B(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
}

void PushButton::up_V(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 1 * car_setting::RemoteForwardSpeed_V;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::down_V(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = -1 * car_setting::RemoteForwardSpeed_V;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::left_V(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 1 * car_setting::RemoteLeftAngularSpeed_V;
    msg.linear.x = 1 * car_setting::RemoteLeftSpeed_V;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::right_V(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = -1 * car_setting::RemoteLeftAngularSpeed_V;
    msg.linear.x = 1 * car_setting::RemoteLeftSpeed_V;
    msg.linear.y = 0;
    msg.linear.z = 0;
}
void PushButton::stop_V(geometry_msgs::msg::Twist &msg)
{
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
}