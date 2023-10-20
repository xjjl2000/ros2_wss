#include "rosserver/RosCallbackInterface.hpp"
#include "rosserver/DelayHz.hpp"
#include "udpserver/UdpSend.hpp"
#include "rclInclude.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class RosImuCallback : public RosCallbackInterface<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu::ConstSharedPtr>
{
        RosDelayHz<sensor_msgs::msg::Imu> imuDelayHz;

        struct timespec lastTime = {0, 0};
        IMU_msg imu_A;
        void callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) override
        {
                // 创建tf2四元数对象
                tf2::Quaternion quaternion;
                quaternion.setX(msg->orientation.x);
                quaternion.setY(msg->orientation.y);
                quaternion.setZ(msg->orientation.z);
                quaternion.setW(msg->orientation.w);
                // 将四元数转换为tf2矩阵
                tf2::Matrix3x3 matrix(quaternion);

                // 计算欧拉角（俯仰、侧倾和航向）
                double roll, pitch, yaw;
                matrix.getRPY(roll, pitch, yaw);
                QString Qroll = QString::number(roll, 'f', 2);
                QString Qpitch = QString::number(pitch, 'f', 2);
                QString Qyaw = QString::number(yaw, 'f', 2);

                QByteArray dataByteArray_Qroll;
                QDataStream dataByteArrayStream_Qroll(&dataByteArray_Qroll, QIODevice::WriteOnly);
                dataByteArrayStream_Qroll << Qroll;
                UdpSendmsg(dataByteArray_Qroll, QString("Qroll"),"localhost",23931);

                QByteArray dataByteArray_Qpitch;
                QDataStream dataByteArrayStream_Qpitch(&dataByteArray_Qpitch, QIODevice::WriteOnly);
                dataByteArrayStream_Qpitch << Qpitch;
                UdpSendmsg(dataByteArray_Qpitch, QString("Qpitch"),"localhost",23932);

                QByteArray dataByteArray_Qyaw;
                QDataStream dataByteArrayStream_Qyaw(&dataByteArray_Qyaw, QIODevice::WriteOnly);
                dataByteArrayStream_Qyaw << Qyaw;
                UdpSendmsg(dataByteArray_Qyaw, QString("Qyaw"),"localhost",23933);
                // ui_->roll->setText(QString::number(roll,'f',2));
                // ui_->pitch->setText(QString::number(pitch,'f',2));
                // ui_->yaw->setText(QString::number(yaw,'f',2));

                imu_A.Acc.x = msg->linear_acceleration.x;
                imu_A.Acc.y = msg->linear_acceleration.y;
                imu_A.Acc.z = msg->linear_acceleration.z;

                imu_A.Angual.x = msg->angular_velocity.x;
                imu_A.Angual.y = msg->angular_velocity.y;
                imu_A.Angual.z = msg->angular_velocity.z;

                if (msg->header.stamp.nanosec)
                {
                        struct timespec timeImu = {0, 0};
                        clock_gettime(CLOCK_REALTIME, &timeImu);
                        if (this->lastTime.tv_sec != 0)
                        {
                                // 如果不等于0，有数据传入
                                // 当前时间减去上一刻时间
                                double hz = 1 / ((timeImu.tv_sec - this->lastTime.tv_sec) + (timeImu.tv_nsec - this->lastTime.tv_nsec) * 1e-9);
                                QString HZ = QString::number(hz, '1', 1);
                                // this->ui_->imuHz_label->setText(QString::number(hz,'1',1));
                        }

                        this->lastTime = timeImu;

                        int t0 = timeImu.tv_sec;
                        int t1 = timeImu.tv_nsec;
                        float imu_delayt = (t0 + t1 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
                        QString delayt = QString::number(imu_delayt, 'f', 3);
                        //     this->ui_->imuTime_label->setText(QString::number(imu_delayt,'f',3));
                }

                

                double delay = imuDelayHz.msgDelay(msg);
                QString Delay = QString::number(delay, 'f', 3);
                double hz = imuDelayHz.msgHz(msg);
                QString HZ = QString::number(hz, '1', 1);
                std::cout << "delay:" << delay << std::endl;

                std::ofstream outfile("imuPackagesLost.txt", std::ios::app); // 创建一个名为.txt 的输出文件流
                outfile << "sec:" << msg->header.stamp.sec << "    nanosec:" << msg->header.stamp.nanosec << "   delay:"<<delay<<"  hz:"<<hz<<"    id:" << msg->header.frame_id << std::endl;
        }

        

        public:
        RosImuCallback(const std::string topic,rclcpp::Node* node) : RosCallbackInterface<sensor_msgs::msg::Imu, sensor_msgs::msg::Imu::ConstSharedPtr>(topic, node){};
};

class RosImuMagneticFieldCallback : public RosCallbackInterface<sensor_msgs::msg::MagneticField, sensor_msgs::msg::MagneticField::ConstSharedPtr>
{
        // RosDelayHz<sensor_msgs::msg::MagneticField> imuMagDelayHz;

        struct timespec lastTime = {0, 0};
        IMU_msg imu_A;
        void callback(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) override
        {
                imu_A.Mag.x = msg->magnetic_field.x;
                imu_A.Mag.y = msg->magnetic_field.y;
                imu_A.Mag.z = msg->magnetic_field.z;
        }

        public:
        RosImuMagneticFieldCallback(const std::string topic,rclcpp::Node* node) : RosCallbackInterface<sensor_msgs::msg::MagneticField, sensor_msgs::msg::MagneticField::ConstSharedPtr>(topic, node){};
};