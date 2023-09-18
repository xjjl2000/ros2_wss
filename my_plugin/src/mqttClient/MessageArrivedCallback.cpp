#include "my_plugin/mqttClient/MessageArrivedCallback.hpp"
#include <QWidget>
#include <QImage>
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs.pb.h"
#include "sensors_msg.pb.h"
#include "pointcloud2.pb.h"
#include "Imu_msgs.pb.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

QlabelShowMessageArrivedCallback::QlabelShowMessageArrivedCallback(QLabel *label_, QLabel *label1_, QLabel *label2_)
{
  if (label_ == nullptr || label1_ == nullptr || label2_ == nullptr)
    throw std::runtime_error("you use a nullprt");
  this->label = label_;
  this->label1 = label1_;
  this->label2 = label2_;
}

void QlabelShowMessageArrivedCallback::func(mqtt::const_message_ptr msg)
{
  if (!strcmp(msg->get_topic().c_str(), "mqtt_cmd_vel"))
  {
    dealCmdvelMessages(msg, label, label1);
  }

  if (!strcmp(msg->get_topic().c_str(), "mqtt_image"))
  {
    dealImageMessages(msg, label, label1, label2);
  }

  if (!strcmp(msg->get_topic().c_str(), "mqtt_imu"))
  {
    dealImuMessages(msg, label, label1, label2);
  }
  if (!strcmp(msg->get_topic().c_str(), "mqtt_pointcloud2"))
  {
    dealCloudMessages(msg, label, label1);
  }
}

void QlabelShowMessageArrivedCallback::dealCmdvelMessages(mqtt::const_message_ptr msg, QLabel *label, QLabel *label1)
{
  geometry_msgs::Twist twist_msg;
  twist_msg.ParseFromArray(msg->get_payload().c_str(), msg->to_string().size());
  label->setText(QString::number(twist_msg.linear_x(), 'f', 2));
  label1->setText(QString::number(twist_msg.angular_z(), 'f', 2));
  std::cout << "FROM topic: " << msg->get_topic() << "  msg:" << msg->get_payload().c_str() << std::endl;
  std::cout << "twist_msg: linear_x" << twist_msg.linear_x() << "  linear_y " << twist_msg.linear_y() << " size:" << twist_msg.ByteSize() << std::endl;
}

void QlabelShowMessageArrivedCallback::dealImageMessages(mqtt::const_message_ptr msg, QLabel *label, QLabel *label1, QLabel *label2)
{
  sensors_msg::ImageProto msg_pb;
  msg_pb.ParseFromString(msg->get_payload());
  // msg.
  std::cout << "FROM topic: " << msg->get_topic() << "  msg_size:" << msg->get_payload().size() << "  msg_pb size:" << msg_pb.data().size() << std::endl;

  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t0 = t.tv_sec, t1 = t.tv_nsec;
  sensor_msgs::msg::CompressedImage imgc;
  imgc.set__format(msg_pb.format());
  imgc.header.frame_id = msg_pb.header_frame_id();
  imgc.header.stamp.nanosec = msg_pb.header_stamp_nanosec();
  imgc.header.stamp.sec = msg_pb.header_stamp_sec();
  // imgc.data.resize(msg_pb.data().size());
  imgc.data.assign(msg_pb.data().begin(), msg_pb.data().end());
  // memcpy(imgc.data.data(),msg_pb.data().data(),msg_pb.data().size());

  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(imgc, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgCallback;
    cv_ptr_compressed->image.copyTo(imgCallback);
    // imgCallback = cv_ptr_compressed->image;
    QImage img = convertToQImage(imgCallback);
    // img.bits();
    // std::cout<<"68:mqtt set\n";
    auto t = QPixmap::fromImage(img).scaled(label->width(), label->height());
    if (cv_ptr_compressed == nullptr)
      throw std::runtime_error("mqtt 73 nullptr");
    // std::cout<<"68:mqtt set t\n";
    // qMutex.lock();
    // label->setPixmap(t);
    // emit updateLabel(label,t);
    // qMutex.unlock();
    // ui_->label_image_back->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back->width(),ui_->label_image_back->height()));
  }
  catch (cv_bridge::Exception &e)
  {
    // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  this->image_Bt_b.nanosec = msg_pb.header_stamp_nanosec();
  if (message_count == 0)
  {
    clock_gettime(CLOCK_REALTIME, &start_time);
  }
  message_count++;
  double hz = message_count / ((t.tv_sec + t.tv_nsec * 1e-9) - (start_time.tv_sec + start_time.tv_nsec * 1e-9));
  label2->setText(QString::number(hz, '1', 1));
  if (this->image_Bt_b.nanosec)
  {
    // struct timespec  imgT = {0, 0};
    // clock_gettime(CLOCK_REALTIME, & imgT);
    // if(this->lastTime.tv_sec!=0){
    //     double hz=1/(( imgT.tv_nsec-this->lastTime.tv_nsec)*1e-9+( imgT.tv_sec-this->lastTime.tv_sec));

    //     label2->setText(QString::number(hz,'1',1));
    // }
    // this->lastTime= imgT;

    double image_front_delayt = (t0 + t1 * 1e-9) - (imgc.header.stamp.sec + imgc.header.stamp.nanosec * 1e-9);
    delay += image_front_delayt;
    double aver_delayt = delay / message_count;
    label1->setText(QString::number(aver_delayt, 'f', 3));
  }
}

void QlabelShowMessageArrivedCallback::dealCloudMessages(mqtt::const_message_ptr msg, QLabel *label, QLabel *label1)
{
  pointcloud2::pointCloud2_pb msg_pb;
  msg_pb.ParseFromString(msg->get_payload());
  sensor_msgs::msg::PointCloud2 cloud;
  using PointField = sensor_msgs::msg::PointField;
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(6);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = PointField::UINT8;
  cloud.point_step = msg_pb.point_step();
  cloud.width = msg_pb.width();
  cloud.row_step = msg_pb.row_step();
  cloud.header.frame_id = msg_pb.frame_id();
  cloud.header.stamp.nanosec = msg_pb.stamp_nanosec();
  cloud.header.stamp.sec = msg_pb.stamp_sec();
  cloud.is_bigendian = msg_pb.is_bigendian();
  cloud.is_dense = msg_pb.is_dense();
  cloud.data.resize(msg_pb.data().size());
  memcpy(cloud.data.data(), msg_pb.data().data(), msg_pb.data().size());

  this->cloud_t.nanosec = msg_pb.stamp_nanosec();

  if (this->cloud_t.nanosec)
  {

    struct timespec pC2_pb_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &pC2_pb_t);
    double time = (pC2_pb_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (pC2_pb_t.tv_sec - this->lastTime.tv_sec);
    if (this->lastTime.tv_sec != 0)
    {
      double hz = 1 / time;
      label1->setText(QString::number(hz, '1', 1));
    }
    this->lastTime = pC2_pb_t;

    int t0 = pC2_pb_t.tv_sec;
    int t1 = pC2_pb_t.tv_nsec;
    double pC2_pb_delayt = (t0 + t1 * 1e-9) - (msg_pb.stamp_sec() + msg_pb.stamp_nanosec() * 1e-9);
    label->setText(QString::number(pC2_pb_delayt, 'f', 3));
  }

  // 发布点云
}

void QlabelShowMessageArrivedCallback::dealImuMessages(mqtt::const_message_ptr msg, QLabel *label, QLabel *label1, QLabel *label2)
{
  Imu_msgs::Imu msg_pb;
  msg_pb.ParseFromString(msg->get_payload());

  sensor_msgs::msg::Imu imumsg;
  imumsg.angular_velocity.set__x(msg_pb.angular_velocity().x());
  imumsg.angular_velocity.set__y(msg_pb.angular_velocity().y());
  imumsg.angular_velocity.set__z(msg_pb.angular_velocity().z());
  imumsg.linear_acceleration.set__x(msg_pb.linear_acceleration().x());
  imumsg.linear_acceleration.set__y(msg_pb.linear_acceleration().y());
  imumsg.linear_acceleration.set__z(msg_pb.linear_acceleration().z());
  imumsg.orientation.set__x(msg_pb.orientation().qx());
  imumsg.orientation.set__y(msg_pb.orientation().qy());
  imumsg.orientation.set__z(msg_pb.orientation().qz());
  imumsg.orientation.set__w(msg_pb.orientation().qw());
  imumsg.header.frame_id = msg_pb.header().frame_id();
  imumsg.header.stamp.nanosec = msg_pb.header().header_stamp_nanosec();
  imumsg.header.stamp.sec = msg_pb.header().header_stamp_sec();
  std::copy(msg_pb.angular_velocity_covariance().begin(), msg_pb.angular_velocity_covariance().end(), imumsg.angular_velocity_covariance.begin());
  std::copy(msg_pb.linear_acceleration_covariance().begin(), msg_pb.linear_acceleration_covariance().end(), imumsg.linear_acceleration_covariance.begin());
  std::copy(msg_pb.orientation_covariance().begin(), msg_pb.orientation_covariance().end(), imumsg.orientation_covariance.begin());

  tf2::Quaternion quaternion;
  quaternion.setX(imumsg.orientation.x);
  quaternion.setY(imumsg.orientation.y);
  quaternion.setZ(imumsg.orientation.z);
  quaternion.setW(imumsg.orientation.w);
  // 将四元数转换为tf2矩阵
  tf2::Matrix3x3 matrix(quaternion);

  // 计算欧拉角（俯仰、侧倾和航向）
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  label->setText(QString::number(roll, 'f', 2));
  label1->setText(QString::number(pitch, 'f', 2));
  label2->setText(QString::number(yaw, 'f', 2));
}

QImage QlabelShowMessageArrivedCallback::convertToQImage(const cv::Mat &image)
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