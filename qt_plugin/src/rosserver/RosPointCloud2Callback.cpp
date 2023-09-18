#include "rosserver/RosCallbackInterface.hpp"
#include "rosmessage/pointcloud2msg.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "handleserver/MyPoint.hpp"
#include "QVector"


class RosPointCloud2Callback : public RosCallbackInterface<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan::ConstSharedPtr>
{

        void callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) override
        {
                QVector<MyPoint> v;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                uint8_t r(120), g(120), b(120);
                // std::cout << msg->ranges.size() << '\n';
                for (size_t i = 0; i < msg->ranges.size(); ++i)
                {
                        pcl::PointXYZRGB point;
                        float range = msg->ranges[i];
                        float angle = msg->angle_min + i * msg->angle_increment;

                        // pcl::PointXYZ point;
                        //  point.x = range * cos(angle)*10;
                        //  point.y = range * sin(angle)*100;
                        //  point.z = 0;
                        //      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        //                      static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                        //      point.rgb = *reinterpret_cast<float *>(&rgb);
                        //      point_cloud_ptr->points.push_back(point);
                        MyPoint p;
                        p.x = range * cos(angle);
                        p.y = range * sin(angle);
                        p.z = 0;
                        //std::cout << "point.x :" << p.x << "point.y :" << p.y << "point.z :" << p.z << '\n';
                        v.push_back(p);
                }

                // point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
                // point_cloud_ptr->height = 1;
                QByteArray dataByteArray;
                QDataStream dataByteArrayStream(&dataByteArray, QIODevice::WriteOnly);
                dataByteArrayStream << v;
                dataByteArray = qCompress(dataByteArray, -1);
                UdpMessage udpMessage;
                udpMessage.topic = QString("point");
                udpMessage.data = std::move(dataByteArray);

                QByteArray msgByteArray;
                QDataStream msgByteArrayStream(&msgByteArray, QIODevice::WriteOnly);
                msgByteArrayStream << udpMessage;

                QUdpSocket udpSocket;
                udpSocket.bind(QHostAddress::Any); // 绑定本地地址和端口号

                // 发送序列化后的数据
                int ans = udpSocket.writeDatagram(msgByteArray, QHostAddress("localhost"), 23912); // 将目标 IP 和端口指定为接收端的地址和端口
                //std::cout << msgByteArray.size() << '\n';
                std::cout << ans << '\n';
        }
        public:
                RosPointCloud2Callback(const std::string topic,rclcpp::Node* node):RosCallbackInterface<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan::ConstSharedPtr>(topic,node){};

};