#include "rosserver/RosCallbackInterface.hpp"
#include "rosserver/DelayHz.hpp"
#include "handleserver/utils.hpp"
#include "rclInclude.hpp"
#include "udpserver/UdpSend.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensors_msg.pb.h"
#include "QImage"
#include "qpixmap.h"

class RosImageFrontCallback : public RosCallbackInterface<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage::ConstSharedPtr>, public RosDelayHz<sensor_msgs::msg::CompressedImage>
{
        struct timespec lastTime = {0, 0};
        int messagecount = 0;
        struct timespec start_time;
        double delay = 0;
        RosDelayHz<sensor_msgs::msg::CompressedImage> image_t;
        void callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) override
        {
                try
                {
                        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                        cv::Mat imgCallback;
                        // if (cv_ptr_compressed == nullptr)
                        //   throw std::runtime_error("765 nullptr");
                        // imgCallback = cv_ptr_compressed->image;
                        cv_ptr_compressed->image.copyTo(imgCallback);
                        QImage img = Utils::convertToQImage(imgCallback).scaled(200, 150);
                        img.bits();

                        auto t = QPixmap::fromImage(img);

                        // std::cout << "t height:" << t.size().height() << "  width:" << t.size().width() << std::endl;
                        std::cout << "pixmap ready\n";
                        QByteArray dataByteArray;
                        QDataStream dataByteArrayStream(&dataByteArray, QIODevice::WriteOnly);
                        dataByteArrayStream << t;

                        UdpUtils::UdpSendmsg(dataByteArray, QString("rosimgFront"),"localhost",23912);
                }
                catch (cv_bridge::Exception &e)
                {
                        // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }

                // delay
                double delay = image_t.msgDelay(msg);
                QString Delay = QString::number(delay, 'f', 3);
                double hz = image_t.msgHz(msg);
                QString HZ = QString::number(hz, '1', 1);
                std::cout << "delay:" << delay << std::endl;

                std::ofstream outfile("imageFrontPackagesLost.txt", std::ios::app); // 创建一个名为.txt 的输出文件流
                outfile << "sec:" << msg->header.stamp.sec << "    nanosec:" << msg->header.stamp.nanosec << "   delay:"<<delay<<"  hz:"<<hz<<"    id:" << msg->header.frame_id << std::endl;
        }

        

public:
        RosImageFrontCallback(const std::string topic, rclcpp::Node *node) : RosCallbackInterface<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage::ConstSharedPtr>(topic, node), RosDelayHz<sensor_msgs::msg::CompressedImage>(){};
};


class RosImageBackCallback : public RosCallbackInterface<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage::ConstSharedPtr>, public RosDelayHz<sensor_msgs::msg::CompressedImage>
{
        struct timespec lastTime = {0, 0};
        int messagecount = 0;
        struct timespec start_time;
        double delay = 0;
        RosDelayHz<sensor_msgs::msg::CompressedImage> image_t;
        void callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) override
        {
                try
                {
                        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
                        cv::Mat imgCallback;
                        // if (cv_ptr_compressed == nullptr)
                        //   throw std::runtime_error("765 nullptr");
                        // imgCallback = cv_ptr_compressed->image;
                        cv_ptr_compressed->image.copyTo(imgCallback);
                        QImage img = Utils::convertToQImage(imgCallback).scaled(265, 150);
                        img.bits();

                        auto t = QPixmap::fromImage(img);

                        // std::cout << "t height:" << t.size().height() << "  width:" << t.size().width() << std::endl;
                        std::cout << "pixmap ready\n";
                        QByteArray dataByteArray;
                        QDataStream dataByteArrayStream(&dataByteArray, QIODevice::WriteOnly);
                        dataByteArrayStream << t;

                        UdpUtils::UdpSendmsg(dataByteArray, QString("rosimgBack"),"localhost",23912);
                }
                catch (cv_bridge::Exception &e)
                {
                        // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }

                // delay
                double delay = image_t.msgDelay(msg);
                QString Delay = QString::number(delay, 'f', 3);
                double hz = image_t.msgHz(msg);
                QString HZ = QString::number(hz, '1', 1);
                std::cout << "delay:" << delay << std::endl;

                std::ofstream outfile("imageBackPackagesLost.txt", std::ios::app); // 创建一个名为.txt 的输出文件流
                outfile << "sec:" << msg->header.stamp.sec << "    nanosec:" << msg->header.stamp.nanosec << "   delay:"<<delay<<"  hz:"<<hz<<"    id:" << msg->header.frame_id << std::endl;
        }

        

public:
        RosImageBackCallback(const std::string topic, rclcpp::Node *node) : RosCallbackInterface<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage::ConstSharedPtr>(topic, node), RosDelayHz<sensor_msgs::msg::CompressedImage>(){};
};