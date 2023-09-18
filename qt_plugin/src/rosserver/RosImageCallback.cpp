#include "rosserver/RosCallbackInterface.hpp"
#include "rclInclude.hpp"

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include <cv_bridge/cv_bridge.h>
#include "sensors_msg.pb.h"
#include "QImage"
#include "qpixmap.h"

class RosImageCallback : public RosCallbackInterface<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage::ConstSharedPtr>
{
        struct timespec lastTime = {0, 0};
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
                        QImage img = convertToQImage(imgCallback).scaled(265, 150);
                        img.bits();

                        auto t = QPixmap::fromImage(img);

                        // std::cout << "t height:" << t.size().height() << "  width:" << t.size().width() << std::endl;
                        std::cout << "pixmap ready\n";
                        QByteArray dataByteArray;
                        QDataStream dataByteArrayStream(&dataByteArray, QIODevice::WriteOnly);
                        dataByteArrayStream << t;

                        UdpMessage udpMessage;
                        udpMessage.topic = QString("rosimg");
                        udpMessage.data = std::move(dataByteArray);

                        QByteArray msgByteArray;
                        QDataStream msgByteArrayStream(&msgByteArray, QIODevice::WriteOnly);
                        msgByteArrayStream << udpMessage;

                        // 创建并配置 UDP Socket
                        QUdpSocket udpSocket;
                        udpSocket.bind(QHostAddress::Any, 23911); // 绑定本地地址和端口号

                        // 发送序列化后的数据
                        int ans = udpSocket.writeDatagram(msgByteArray, QHostAddress("localhost"), 23912); // 将目标 IP 和端口指定为接收端的地址和端口
                        std::cout << ans << '\n';
                }
                catch (cv_bridge::Exception &e)
                {
                        // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }

                // delay

                // hz
                if (msg->header.stamp.nanosec)
                {
                        struct timespec image_back_t = {0, 0};
                        clock_gettime(CLOCK_REALTIME, &image_back_t);
                        if (this->lastTime.tv_sec != 0)
                        {
                                double hz = 1 / ((image_back_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (image_back_t.tv_sec - this->lastTime.tv_sec));
                                // this->ui_->imageBackHz_label->setText(QString::number(hz,'1',1));
                        }
                        this->lastTime = image_back_t;

                        int t0 = image_back_t.tv_sec;
                        int t1 = image_back_t.tv_nsec;
                        double image_back_delayt = (t0 + t1 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
                        // this->ui_->imageBackTime_label->setText(QString::number(image_back_delayt,'f',3));
                }
        }

        QImage convertToQImage(const cv::Mat &image)
        {
                cv::Mat rgb;
                if (image.channels() == 3)
                {
                        // cvt Mat BGR 2 QImage RGB
                        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
                }
                else
                {
                        rgb = image.clone();
                }

                QImage img(rgb.cols, rgb.rows, QImage::Format_RGB888);
                uchar *imageData = img.bits(); // 获取图像数据指针

                // 将像素数据复制到QImage中
                memcpy(imageData, rgb.data, rgb.cols * rgb.rows * 3); // 假设每个像素有三个通道

                return img;
        }

public:
        RosImageCallback(const std::string topic,rclcpp::Node* node) : RosCallbackInterface<sensor_msgs::msg::CompressedImage, sensor_msgs::msg::CompressedImage::ConstSharedPtr>(topic, node){};
};