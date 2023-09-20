#include "rosserver/RosCallbackInterface.hpp"
#include "rosserver/DelayHz.hpp"
#include "rosmessage/pointcloud2msg.hpp"
#include "udpserver/UdpSend.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "handleserver/MyPoint.hpp"
#include "QVector"


class RosPointCloud2Callback : public RosCallbackInterface<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan::ConstSharedPtr>
{
        RosDelayHz<sensor_msgs::msg::LaserScan> scan_t;


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
                double delay = scan_t.msgDelay(msg);
                QString Delay=QString::number(delay, 'f', 3);
                double hz=scan_t.msgHz(msg);
                QString HZ=QString::number(hz, '1', 1);
                std::cout<<"delay:"<<delay<<std::endl;

                std::ofstream outfile("laserScanPackagesLost.txt", std::ios::app); // 创建一个名为.txt 的输出文件流
                outfile << "sec:" << msg->header.stamp.sec << "    nanosec:" << msg->header.stamp.nanosec <<"   delay:"<<delay<<"  hz:"<<hz<< "    id:" << msg->header.frame_id << std::endl;


                QByteArray dataByteArray;
                QDataStream dataByteArrayStream(&dataByteArray, QIODevice::WriteOnly);
                dataByteArrayStream << v;
                dataByteArray = qCompress(dataByteArray, -1);

                UdpSendmsg(dataByteArray, QString("point"),"localhost",23912);
        }
        public:
                RosPointCloud2Callback(const std::string topic,rclcpp::Node* node):RosCallbackInterface<sensor_msgs::msg::LaserScan, sensor_msgs::msg::LaserScan::ConstSharedPtr>(topic,node){};

};