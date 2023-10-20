#include "Rosmsg/imuCallback.hpp"
#include "Imu_msgs.pb.h"

ImuMsg::ImuMsg() : Node("imu_convert")
{
        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.keep_last(10);
        // imu设置qos
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", qos, std::bind(&ImuMsg::imu_callback, this, std::placeholders::_1));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/convert", qos);
}

void ImuMsg::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
        imucount++;
        std::cout<<"imu count:"<<imucount<<std::endl;

        std::string str1 = msg->header.frame_id;
        std::string str2 = std::to_string(imucount);
        std::string str3 = " ";
        std::string id = str1 + "" + str3 + "" + str2;
        struct timespec t = {0, 0};
        clock_gettime(CLOCK_REALTIME, &t);
        int t2 = t.tv_nsec;
        int t1 = t.tv_sec;

        Imu_msgs::Imu imumsg;
        imumsg.mutable_angular_velocity()->set_x(msg->angular_velocity.x);
        imumsg.mutable_angular_velocity()->set_y(msg->angular_velocity.y);
        imumsg.mutable_angular_velocity()->set_z(msg->angular_velocity.z);
        imumsg.mutable_linear_acceleration()->set_x(msg->linear_acceleration.x);
        imumsg.mutable_linear_acceleration()->set_y(msg->linear_acceleration.y);
        imumsg.mutable_linear_acceleration()->set_z(msg->linear_acceleration.z);
        imumsg.mutable_header()->set_frame_id(msg->header.frame_id);
        imumsg.mutable_header()->set_header_stamp_sec(t1);
        imumsg.mutable_header()->set_header_stamp_nanosec(t2);
        imumsg.mutable_orientation()->set_qx(msg->orientation.x);
        imumsg.mutable_orientation()->set_qy(msg->orientation.y);
        imumsg.mutable_orientation()->set_qz(msg->orientation.z);
        imumsg.mutable_orientation()->set_qw(msg->orientation.w);
        imumsg.add_angular_velocity_covariance(*msg->angular_velocity_covariance.data());
        imumsg.add_linear_acceleration_covariance(*msg->linear_acceleration_covariance.data());
        imumsg.add_orientation_covariance(*msg->orientation_covariance.data());

        std::string ser_msg;
        imumsg.SerializeToString(&ser_msg);

        //   mqtt_imu_pub.pub("mqtt_imu", ser_msg, 0);

        sensor_msgs::msg::Imu RosImu;
        RosImu.set__angular_velocity(msg->angular_velocity);
        RosImu.set__angular_velocity_covariance(msg->angular_velocity_covariance);
        RosImu.header.frame_id = id;
        RosImu.header.stamp.sec = t1;
        RosImu.header.stamp.nanosec = t2;
        RosImu.set__linear_acceleration(msg->linear_acceleration);
        RosImu.set__linear_acceleration_covariance(msg->linear_acceleration_covariance);
        RosImu.set__orientation(msg->orientation);
        RosImu.set__orientation_covariance(msg->orientation_covariance);

        imu_pub_->publish(RosImu);
}