#include "rosmessage/location.hpp"

LocationMsg::LocationMsg() : Node("location"), count_(0)
{
        rclcpp::QoS qos(10);
        qos = qos.best_effort();
        qos = qos.keep_last(10);
        // uwb-学校
        //  point_uwb=node_->create_subscription<geometry_msgs::msg::Pose>("/uwb_locationss",qos,std::bind(&LocationMsg::uwb_poseA_callback,this,std::placeholders::_1));
        point_ubw_B = this->create_subscription<geometry_msgs::msg::Pose2D>("/location_2D", qos, std::bind(&LocationMsg::uwb_poseB_callback, this, std::placeholders::_1));

        // april_tag-29
        point = this->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose", qos, std::bind(&LocationMsg::poseA_callback, this, std::placeholders::_1));
        point_B = this->create_subscription<geometry_msgs::msg::PoseStamped>("/uwb_location", qos, std::bind(&LocationMsg::poseB_callback, this, std::placeholders::_1));
}

void LocationMsg::uwb_poseA_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg)
{
        poseA.x = msg->position.x;
        poseA.y = msg->position.y;
        // pointreceieved();
}
void LocationMsg::uwb_poseB_callback(const geometry_msgs::msg::Pose2D::ConstSharedPtr msg)
{
        poseB.x = msg->x;
        poseB.y = msg->y;
        // pointreceieved();
}

void LocationMsg::poseA_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
        poseA.x = msg->pose.position.x;
        poseA.y = msg->pose.position.y;
        // pointreceieved();
}
void LocationMsg::poseB_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
{
        poseB.x = msg->pose.position.x;
        poseB.y = msg->pose.position.y;
        // pointreceieved();
}