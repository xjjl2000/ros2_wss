#include "rosmessage/imumsg.hpp"


ImuMsg::ImuMsg() : Node("imumsg"), count_(0)
{
    actualDataLine_sub_A = this->create_subscription<sensor_msgs::msg::Imu>("/imu/data_A", 10, std::bind(&ImuMsg::imudata_callback, this, std::placeholders::_1));
    actualMag_data_sub_A = this->create_subscription<sensor_msgs::msg::MagneticField>("/imu/mag", 10, std::bind(&ImuMsg::imumag_callback, this, std::placeholders::_1));

    actualDataLine_sub_B = this->create_subscription<sensor_msgs::msg::Imu>("/imuuu", 10, std::bind(&ImuMsg::imudata_callback_B, this, std::placeholders::_1));
    actualMag_data_sub_B = this->create_subscription<sensor_msgs::msg::MagneticField>("/imu/mag_B", 10, std::bind(&ImuMsg::imumag_callback_B, this, std::placeholders::_1));
}


void ImuMsg::imudata_callback(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line)
{
    // 创建tf2四元数对象
    tf2::Quaternion quaternion;
    quaternion.setX(actualData_line->orientation.x);
    quaternion.setY(actualData_line->orientation.y);
    quaternion.setZ(actualData_line->orientation.z);
    quaternion.setW(actualData_line->orientation.w);
    // 将四元数转换为tf2矩阵
    tf2::Matrix3x3 matrix(quaternion);

    // 计算欧拉角（俯仰、侧倾和航向）
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    // ui_->roll->setText(QString::number(roll,'f',2));
    // ui_->pitch->setText(QString::number(pitch,'f',2));
    // ui_->yaw->setText(QString::number(yaw,'f',2));

    imu_A.Acc.x = actualData_line->linear_acceleration.x;
    imu_A.Acc.y = actualData_line->linear_acceleration.y;
    imu_A.Acc.z = actualData_line->linear_acceleration.z;

    imu_A.Angual.x = actualData_line->angular_velocity.x;
    imu_A.Angual.y = actualData_line->angular_velocity.y;
    imu_A.Angual.z = actualData_line->angular_velocity.z;

    this->imu_At.sec = actualData_line->header.stamp.sec;
    this->imu_At.nanosec = actualData_line->header.stamp.nanosec;

    if (this->imu_At.nanosec)
    {
        struct timespec timeImu = {0, 0};
        clock_gettime(CLOCK_REALTIME, &timeImu);
        if (this->lastTime2.tv_sec != 0)
        {
            // 如果不等于0，有数据传入
            // 当前时间减去上一刻时间
            double hz = 1 / ((timeImu.tv_sec - this->lastTime2.tv_sec) + (timeImu.tv_nsec - this->lastTime2.tv_nsec) * 1e-9);
            QString HZ=QString::number(hz,'1',1);
            // this->ui_->imuHz_label->setText(QString::number(hz,'1',1));
        }

        this->lastTime2 = timeImu;

        int t0 = timeImu.tv_sec;
        int t1 = timeImu.tv_nsec;
        float imu_delayt = (t0 + t1 * 1e-9) - (actualData_line->header.stamp.sec + actualData_line->header.stamp.nanosec * 1e-9);
        QString delayt=QString::number(imu_delayt,'f',3);
        //     this->ui_->imuTime_label->setText(QString::number(imu_delayt,'f',3));
    }
}
void ImuMsg::imumag_callback(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line)
{
    imu_A.Mag.x = actualMagData_line->magnetic_field.x;
    imu_A.Mag.y = actualMagData_line->magnetic_field.y;
    imu_A.Mag.z = actualMagData_line->magnetic_field.z;
}

void ImuMsg::imudata_callback_B(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line)
{
    // 创建tf2四元数对象
    tf2::Quaternion quaternion;
    quaternion.setX(actualData_line->orientation.x);
    quaternion.setY(actualData_line->orientation.y);
    quaternion.setZ(actualData_line->orientation.z);
    quaternion.setW(actualData_line->orientation.w);
    // 将四元数转换为tf2矩阵
    tf2::Matrix3x3 matrix(quaternion);

    // 计算欧拉角（俯仰、侧倾和航向）
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    // ui_->roll_B->setText(QString::number(roll,'f',2));
    // ui_->pitch_B->setText(QString::number(pitch,'f',2));
    // ui_->yaw_B->setText(QString::number(yaw,'f',2));

    imu_B.Acc.x = actualData_line->linear_acceleration.x;
    imu_B.Acc.y = actualData_line->linear_acceleration.y;
    imu_B.Acc.z = actualData_line->linear_acceleration.z;

    imu_B.Angual.x = actualData_line->angular_velocity.x;
    imu_B.Angual.y = actualData_line->angular_velocity.y;
    imu_B.Angual.z = actualData_line->angular_velocity.z;

    this->imu_At.sec = actualData_line->header.stamp.sec;
    this->imu_At.nanosec = actualData_line->header.stamp.nanosec;

    if (this->imu_At.nanosec)
    {
        struct timespec timeImu = {0, 0};
        clock_gettime(CLOCK_REALTIME, &timeImu);
        if (this->lastTime.tv_sec != 0)
        {
            // 如果不等于0，有数据传入
            // 当前时间减去上一刻时间
            double hz = 1 / ((timeImu.tv_sec - this->lastTime.tv_sec) + (timeImu.tv_nsec - this->lastTime.tv_nsec) * 1e-9);
            QString HZ=QString::number(hz,'1',1);
            
            // this->ui_->imuHz_label_B->setText(QString::number(hz,'1',1));
            std::ofstream outfile("imu.txt", std::ios::app); // 创建一个名为.txt 的输出文件流
            outfile << "sec：" << actualData_line->header.stamp.sec << "    nanosec：" << actualData_line->header.stamp.nanosec << "    hz:" << hz
                    << "   lastTime: " << this->lastTime.tv_nsec << " " << this->lastTime.tv_sec << "    now time " << timeImu.tv_nsec << " " << timeImu.tv_sec << std::endl;
        }

        this->lastTime = timeImu;

        int t0 = timeImu.tv_sec;
        int t1 = timeImu.tv_nsec;
        float imu_delayt = (t0 + t1 * 1e-9) - (actualData_line->header.stamp.sec + actualData_line->header.stamp.nanosec * 1e-9);
        QString delayt=QString::number(imu_delayt,'f',3);
        
        //     this->ui_->imuTime_label_B->setText(QString::number(imu_delayt,'f',3));
    }
}
void ImuMsg::imumag_callback_B(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line)
{
    imu_B.Mag.x = actualMagData_line->magnetic_field.x;
    imu_B.Mag.y = actualMagData_line->magnetic_field.y;
    imu_B.Mag.z = actualMagData_line->magnetic_field.z;
}
