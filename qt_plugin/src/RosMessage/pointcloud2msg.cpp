#include "rosmessage/pointcloud2msg.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "handleserver/MyPoint.hpp"

PointCloudMsg::PointCloudMsg() : Node("cloud2"), count_(0)
{

  pointcloud2_sub_A = this->create_subscription<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10, std::bind(&PointCloudMsg::pointcloud2_callback, this, std::placeholders::_1));
  
  // laser=this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&PointCloudMsg::scan_callback,this,std::placeholders::_1));
  
  pb_2_cloud2_A = this->create_publisher<sensor_msgs::msg::PointCloud2>("pb_2_Pointcloud2", 10);
  pb_2_cloud2_B = this->create_publisher<sensor_msgs::msg::PointCloud2>("pb_2_Pointcloud2_B", 10);
}

void PointCloudMsg::pointcloud2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg<pcl::PointXYZI>(*pointcloud2_msg,*point_cloud_ptr);
  std::cout<<"/livox/lidar msg size:"<<pointcloud2_msg->data.size()<<std::endl;
  this->cloud_At.sec = pointcloud2_msg->header.stamp.sec;
  this->cloud_At.nanosec = pointcloud2_msg->header.stamp.nanosec;

  if (this->cloud_At.nanosec)
  {
    struct timespec pC2_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &pC2_t);
    double time = pointcloud2_msg->header.stamp.sec + pointcloud2_msg->header.stamp.nanosec * 1e-9;
    if (this->lastTime.tv_sec != 0)
    {
      double hz = 1 / (pC2_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (pC2_t.tv_sec - this->lastTime.tv_sec);
      QString HZ=QString::number(hz,'1',1);
      
      // this->ui_->cloudHz_label->setText(QString::number(hz,'1',1));
    }
    this->lastTime = pC2_t;

    int t0 = pC2_t.tv_sec;
    int t1 = pC2_t.tv_nsec;
    double pC2_delayt = (t0 + t1 * 1e-9) - (pointcloud2_msg->header.stamp.sec + pointcloud2_msg->header.stamp.nanosec * 1e-9);
    QString delayt=QString::number(pC2_delayt,'f',3);
    
    // this->ui_->cloudTime_label->setText(QString::number(pC2_delayt,'f',3));
  }
}

void PointCloudMsg::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg){

  QVector<MyPoint> v;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        uint8_t r(120), g(120), b(120);
         //std::cout << msg->ranges.size() << '\n';
        for (size_t i = 0; i < msg->ranges.size(); ++i){
            pcl::PointXYZRGB point;
            float range = msg->ranges[i];
            float angle = msg->angle_min + i * msg->angle_increment;

            //pcl::PointXYZ point;
            // point.x = range * cos(angle)*10;
            // point.y = range * sin(angle)*100;
            // point.z = 0;
            //     uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
            //                     static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            //     point.rgb = *reinterpret_cast<float *>(&rgb);
            //     point_cloud_ptr->points.push_back(point);
            MyPoint p;
            p.x=range * cos(angle);
            p.y=range * sin(angle);
            p.z=0;
            std::cout<<"point.x :"<<p.x<<"point.y :"<<p.y<<"point.z :"<<p.z <<'\n';
            v.push_back(p);
        }
        
        // point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
        // point_cloud_ptr->height = 1;
      QByteArray dataByteArray;
      QDataStream dataByteArrayStream(&dataByteArray, QIODevice::WriteOnly);
      dataByteArrayStream << v;
      dataByteArray=qCompress(dataByteArray,-1);
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
      std::cout<< msgByteArray.size()<<'\n';
      std::cout << ans << '\n';



}