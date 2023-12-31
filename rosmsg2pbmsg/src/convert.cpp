#include "convert.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs.pb.h"
#include "Imu_msgs.pb.h"
#include "laserscan.pb.h"
#include "callback/ActionCallback.hpp"
#include "callback/ConnectCallback.hpp"
#include "callback/MessageArrivedCallback.hpp"
#include "callback/DeliveryComplete.hpp"

using namespace std::chrono_literals;

Convert::Convert() : Node("listener"), count_(0)
{
  rclcpp::QoS qos(10);
  qos.best_effort();
  qos.keep_last(10);

  mqtt::connect_options sub_conn_opts;
  sub_conn_opts.set_clean_session(false);

  auto actionCallbackPtr = std::make_shared<CompentsActionCallback>();
  auto cac_ptr = actionCallbackPtr.get();
  std::string sub_name = "ActionCallback";
  cac_ptr->setFailureFunc(std::make_shared<FailureCallback>(sub_name));
  cac_ptr->setSuccessFunc(std::make_shared<SuccessCallback>(sub_name));
  auto sub_callback_ptr = std::make_shared<CompentsCallback>();
  auto sub_callback_ptr_r = sub_callback_ptr.get();
  sub_callback_ptr_r->setConnectedFunc(std::make_shared<PrintConnectedCallback>());
  sub_callback_ptr_r->setConnectionLostFunc(std::make_shared<ReConnectConnectionLostCallback>(actionCallbackPtr));
  // sub_callback_ptr_r->setMessageArrivedFunc(std::make_shared<QlabelShowMessageArrivedCallback>(nullptr));

  mqtt_image_sub = MyMqttClient("mqttpublish_img_sub", "192.168.31.107", 1884,
                                sub_conn_opts,
                                sub_callback_ptr,
                                actionCallbackPtr);
  // mqtt_image_sub.sub("mqtt_image",0);

  // mqtt_cmdvel_sub=MyMqttClient("cmd_mqtt","192.168.31.107",1884,
  //                           sub_conn_opts,
  //                           sub_callback_ptr,
  //                           actionCallbackPtr);
  // mqtt_cmdvel_sub.sub("mqtt_cmd_vel",0);

//imgback
  auto pub_conn_opts = mqtt::connect_options_builder()
                           .clean_session()
                           .will(mqtt::message("mqttpublish", "Last will and testament.", 0))
                           .finalize();
  auto pub_callback_ptr = std::make_shared<CompentsCallback>();
  auto pub_callback_ptr_r = pub_callback_ptr.get();
  pub_callback_ptr_r->setConnectionLostFunc(std::make_shared<PrintConnectionLostCallback>());
  pub_callback_ptr_r->setDeliveryCompleteFunc(std::make_shared<PrintDeliveryCompleteCallback>());
  pub_callback_ptr_r->setConnectedFunc(std::make_shared<PrintConnectedCallback>());

  mqtt_imageback_pub = MyMqttClient("mqtt_pub_imageback", "192.168.31.107", 1884,
                                pub_conn_opts,
                                pub_callback_ptr);

//imgfront
  auto imgfront_pub_callback_ptr = std::make_shared<CompentsCallback>();
  auto imgfront_pub_callback_ptr_r = imgfront_pub_callback_ptr.get();
  imgfront_pub_callback_ptr_r->setConnectionLostFunc(std::make_shared<PrintConnectionLostCallback>());
  imgfront_pub_callback_ptr_r->setDeliveryCompleteFunc(std::make_shared<PrintDeliveryCompleteCallback>());
  imgfront_pub_callback_ptr_r->setConnectedFunc(std::make_shared<PrintConnectedCallback>());

  mqtt_imagefront_pub = MyMqttClient("mqtt_pub_imagefront", "192.168.31.107", 1884,
                                pub_conn_opts,
                                imgfront_pub_callback_ptr);

//imu
  auto imu_pub_callback_ptr = std::make_shared<CompentsCallback>();
  auto imu_pub_callback_ptr_r = imu_pub_callback_ptr.get();
  imu_pub_callback_ptr_r->setConnectionLostFunc(std::make_shared<PrintConnectionLostCallback>());
  imu_pub_callback_ptr_r->setDeliveryCompleteFunc(std::make_shared<PrintDeliveryCompleteCallback>());
  imu_pub_callback_ptr_r->setConnectedFunc(std::make_shared<PrintConnectedCallback>());

  mqtt_imu_pub = MyMqttClient("mqtt_pub_imu", "192.168.31.107", 1884,
                              pub_conn_opts,
                              imu_pub_callback_ptr);

//cloud
  auto cloud_pub_callback_ptr = std::make_shared<CompentsCallback>();
  auto cloud_pub_callback_ptr_r = cloud_pub_callback_ptr.get();
  cloud_pub_callback_ptr_r->setConnectionLostFunc(std::make_shared<PrintConnectionLostCallback>());
  cloud_pub_callback_ptr_r->setDeliveryCompleteFunc(std::make_shared<PrintDeliveryCompleteCallback>());
  cloud_pub_callback_ptr_r->setConnectedFunc(std::make_shared<PrintConnectedCallback>());

  mqtt_cloud_pub = MyMqttClient("mqtt_pub_cloud", "192.168.31.107", 1884,
                                pub_conn_opts,
                                cloud_pub_callback_ptr);

//laserscan
  auto laserscan_pub_callback_ptr = std::make_shared<CompentsCallback>();
  auto laserscan_pub_callback_ptr_r = laserscan_pub_callback_ptr.get();
  laserscan_pub_callback_ptr_r->setConnectionLostFunc(std::make_shared<PrintConnectionLostCallback>());
  laserscan_pub_callback_ptr_r->setDeliveryCompleteFunc(std::make_shared<PrintDeliveryCompleteCallback>());
  laserscan_pub_callback_ptr_r->setConnectedFunc(std::make_shared<PrintConnectedCallback>());

  mqtt_laserscan_pub = MyMqttClient("mqtt_pub_laserscan", "192.168.31.107", 1884,
                                pub_conn_opts,
                                laserscan_pub_callback_ptr);

  // Mqtt_pub mqtt_image_b("192.168.31.107",1884);
  // mqtt_image_b.init();
  // mqtt_image_b.pub_message("mqttpublish","6666");

  // std::string cmdmsg = mqtt_imsub.sub("cmd_vel");
  //  mqtt_imsub.init();

  // Mqtt_sub cmd_sub("mqttpublish_cmd","192.168.31.107", 1884,sub_conn_opts);
  // cmd_sub.sub("mqtt_cmd_vel",0);

  sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10, std::bind(&Convert::pointcloud2_sub_callback, this, std::placeholders::_1));
  pub_ = create_publisher<std_msgs::msg::ByteMultiArray>("lidar_pb", 10);

  laserScan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos, std::bind(&Convert::laserScanCallback, this, std::placeholders::_1));
  laserScan_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan/convert", 10);

  CompressedImageBack_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>("/rear_camera_nx/camera/image_raw/compressed", 10, std::bind(&Convert::CompressedImageBack_callback, this, std::placeholders::_1));
  CompressedImageBack_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/rear_camera/camera/image_raw/compressed/convert", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

  CompressedImageFront_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("/d455_camera_nx/color/image_raw/compressed/convert", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
  CompressedImageFront_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>("/d455_camera_nx/color/image_raw/compressed", qos, std::bind(&Convert::CompressedImageFront_callback, this, std::placeholders::_1));
  // image_f_sub_=create_subscription<sensor_msgs::msg::Image>("/d455_camera/color/image_rawww",10,std::bind(&Convert::image_f_callback,this,std::placeholders::_1));

  // imu设置qos
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", qos, std::bind(&Convert::imu_callback, this, std::placeholders::_1));
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/convert", qos);
}

void Convert::pointcloud2_sub_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
{
  // 获取当前时间
  rclcpp::Time now = this->get_clock()->now();
  builtin_interfaces::msg::Time tt = now;
  int n1 = tt.sec;
  int n2 = tt.nanosec;

  double pC2_pb_delayt = (n1 + n2 * 1e-9) - (cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9);

  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t2 = t.tv_nsec;
  int t1 = t.tv_sec;
  pointcloud2::pointCloud2_pb pbmsg;
  pbmsg.set_frame_id(cloud->header.frame_id);
  pbmsg.set_height(cloud->height);
  pbmsg.set_width(cloud->width);
  pbmsg.set_point_step(cloud->point_step);
  pbmsg.set_row_step(cloud->row_step);
  pbmsg.set_stamp_nanosec(t2);
  pbmsg.set_stamp_sec(t1);
  pbmsg.set_is_bigendian(false);
  pbmsg.set_is_dense(true);
  pbmsg.set_convert_time(pC2_pb_delayt);
  pbmsg.mutable_data()->resize(cloud->width * cloud->point_step);
  memcpy(const_cast<void *>(static_cast<const void *>(pbmsg.data().data())), cloud->data.data(), cloud->width * cloud->point_step);

  const auto ser_start_t = std::chrono::high_resolution_clock::now();
  std::string ser_msg = pbmsg.SerializeAsString();
  const auto ser_end_t = std::chrono::high_resolution_clock::now();
  const auto ser_t = std::chrono::duration_cast<std::chrono::nanoseconds>(ser_end_t - ser_start_t).count();

  // 将buffer转换为vector<uint8_t>
  std::vector<uint8_t> vector_data(ser_msg.begin(), ser_msg.end());
  // 创建std_msgs::msg::ByteMultiArray消息，并将data添加到它的data字段
  std_msgs::msg::ByteMultiArray buff_msg;
  buff_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  buff_msg.layout.dim[0].size = vector_data.size();
  buff_msg.layout.dim[0].stride = 1;
  buff_msg.layout.dim[0].label = "my_data";
  buff_msg.data = std::move(vector_data);

  mqtt_cloud_pub.pub("mqtt_pointcloud2", ser_msg, 0);

  // pub_->publish(buff_msg);
}

void Convert::laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  laserscan_count++;

  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t2 = t.tv_nsec;
  int t1 = t.tv_sec;

  sensor_msgs::msg::LaserScan laserScanmsg;

  std::string str1 = msg->header.frame_id;
  std::string str2 = std::to_string(laserscan_count);
  std::string str3 = " ";
  std::string id = str1 + "" + str3 + "" + str2;

  laserScanmsg.header.set__frame_id(id);
  laserScanmsg.header.stamp.nanosec = t2;
  laserScanmsg.header.stamp.sec = t1;
  laserScanmsg.set__angle_increment(msg->angle_increment);
  laserScanmsg.set__angle_max(msg->angle_max);
  laserScanmsg.set__angle_min(msg->angle_min);
  laserScanmsg.set__intensities(msg->intensities);
  laserScanmsg.set__range_max(msg->range_max);
  laserScanmsg.set__range_min(msg->range_min);
  laserScanmsg.set__ranges(msg->ranges);
  laserScanmsg.set__scan_time(msg->scan_time);
  laserScanmsg.set__time_increment(msg->time_increment);

  // laserScan_pub->publish(laserScanmsg);

  laserscan::laserscanpb lasermsg;
  lasermsg.add_intensities(msg->angle_increment);
  lasermsg.add_ranges(*msg->ranges.data());
  lasermsg.set_angle_max(msg->angle_max);
  lasermsg.set_angle_min(msg->angle_min);
  lasermsg.set_angle_increment(msg->angle_increment);
  lasermsg.set_scan_time(msg->scan_time);
  lasermsg.set_time_increment(msg->time_increment);
  lasermsg.set_range_max(msg->range_max);
  lasermsg.set_range_min(msg->range_min);
  lasermsg.mutable_header()->set_frame_id(msg->header.frame_id);
  lasermsg.mutable_header()->set_header_stamp_nanosec(msg->header.stamp.nanosec);
  lasermsg.mutable_header()->set_header_stamp_sec(msg->header.stamp.sec);
  std::string ser_msg;
  lasermsg.SerializeToString(&ser_msg);
  mqtt_laserscan_pub.pub("mqtt_laserscan",ser_msg,0);
}

void Convert::image_b_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg, MyMqttClient &mqtt_image_b)
{
  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t2 = t.tv_nsec;
  int t1 = t.tv_sec;
  double image_pb_delayt = (t1 + t2 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
  std::cout << ": time=" << image_pb_delayt << std::endl;
  std::cout << "size_image=" << msg->data.size() << std::endl;

  sensors_msg::ImageProto img_pb;
  img_pb.set_header_frame_id(msg->header.frame_id);
  img_pb.set_header_stamp_nanosec(t2);
  img_pb.set_header_stamp_sec(t1);
  img_pb.set_format(msg->encoding);
  img_pb.set_width(msg->width);
  img_pb.set_height(msg->height);
  img_pb.set_data(msg->data.data(), msg->data.size());
  img_pb.set_step(msg->width * sensor_msgs::image_encodings::numChannels(msg->encoding));

  const auto pb_start_t = std::chrono::high_resolution_clock::now();
  std::string ser_msg = img_pb.SerializeAsString();
  const auto pb_end_t = std::chrono::high_resolution_clock::now();
  const auto pb_t = std::chrono::duration_cast<std::chrono::nanoseconds>(pb_end_t - pb_start_t).count();
  // std::cout << "size:" << ser_msg.size() << std::endl;

  std::string compressed_data;
  compress_data(ser_msg, compressed_data);
  std::cout << "compressed_size:" << compressed_data.size() << std::endl;
  // mqtt_imageback_pub.pub("mqtt_image", compressed_data,0);

  // // 将buffer转换为vector<uint8_t>
  // std::vector<uint8_t> vector_data(ser_msg.begin(), ser_msg.end());
  // // 创建std_msgs::msg::ByteMultiArray消息，并将data添加到它的data字段
  // std_msgs::msg::ByteMultiArray buff_msg;
  // buff_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  // buff_msg.layout.dim[0].size = vector_data.size();
  // buff_msg.layout.dim[0].stride = 1;
  // buff_msg.layout.dim[0].label = "my_data";
  // buff_msg.data = std::move(vector_data);

  // image_pub_->publish(buff_msg);
}
void Convert::image_f_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  rclcpp::Time now = this->get_clock()->now();
  builtin_interfaces::msg::Time tt = now;
  sensors_msg::ImageProto img_pb;
  img_pb.set_header_frame_id(msg->header.frame_id);
  img_pb.set_header_stamp_nanosec(msg->header.stamp.nanosec);
  img_pb.set_header_stamp_sec(msg->header.stamp.sec);
  img_pb.set_format(msg->encoding);
  img_pb.set_width(msg->width);
  img_pb.set_height(msg->height);
  img_pb.set_data(msg->data.data(), msg->data.size());
  img_pb.set_step(msg->width * sensor_msgs::image_encodings::numChannels(msg->encoding));

  const auto pb_start_t = std::chrono::high_resolution_clock::now();
  std::string ser_msg = img_pb.SerializeAsString();
  const auto pb_end_t = std::chrono::high_resolution_clock::now();
  const auto pb_t = std::chrono::duration_cast<std::chrono::nanoseconds>(pb_end_t - pb_start_t).count();

  // 将buffer转换为vector<uint8_t>
  std::vector<uint8_t> vector_data(ser_msg.begin(), ser_msg.end());
  // 创建std_msgs::msg::ByteMultiArray消息，并将data添加到它的data字段
  std_msgs::msg::ByteMultiArray buff_msg;
  buff_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  buff_msg.layout.dim[0].size = vector_data.size();
  buff_msg.layout.dim[0].stride = 1;
  buff_msg.layout.dim[0].label = "my_data";
  buff_msg.data = std::move(vector_data);

  image_f_pub_->publish(buff_msg);
}

void Convert::CompressedImageFront_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  imageFront_count++;
  std::string str1 = msg->header.frame_id;
  std::string str2 = std::to_string(imageFront_count);
  std::string str3 = " ";
  std::string id = str1 + "" + str3 + "" + str2;

  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t2 = t.tv_nsec;
  int t1 = t.tv_sec;

  sensors_msg::ImageProto msg_pb;
  msg_pb.set_header_frame_id(id);
  msg_pb.set_header_stamp_nanosec(t2);
  msg_pb.set_header_stamp_sec(t1);
  msg_pb.set_format(msg->format);
  msg_pb.set_data(msg->data.data(), msg->data.size());
  std::string ser_msg;
  msg_pb.SerializeToString(&ser_msg);
  mqtt_imagefront_pub.pub("mqtt_imageFront", ser_msg, 0);

  sensor_msgs::msg::CompressedImage Compressedimage_msg;
  Compressedimage_msg.header.frame_id = id;
  Compressedimage_msg.header.stamp.nanosec = t2;
  Compressedimage_msg.header.stamp.sec = t1;
  Compressedimage_msg.set__format(msg->format);
  Compressedimage_msg.data.resize(msg->data.size());
  memcpy(Compressedimage_msg.data.data(), msg->data.data(), msg->data.size());

  // CompressedImageFront_pub_->publish(Compressedimage_msg);
}
void Convert::CompressedImageBack_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  imageBack_count++;
  std::string str1 = msg->header.frame_id;
  std::string str2 = std::to_string(imageBack_count);
  std::string str3 = " ";
  std::string id = str1 + "" + str3 + "" + str2;
  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t2 = t.tv_nsec;
  int t1 = t.tv_sec;

  sensors_msg::ImageProto msg_pb;
  msg_pb.set_header_frame_id(id);
  msg_pb.set_header_stamp_nanosec(t2);
  msg_pb.set_header_stamp_sec(t1);
  msg_pb.set_format(msg->format);
  msg_pb.set_data(msg->data.data(), msg->data.size());
  std::string ser_msg;
  msg_pb.SerializeToString(&ser_msg);
  mqtt_imageback_pub.pub("mqtt_imageBack", ser_msg, 0);


  sensor_msgs::msg::CompressedImage Compressedimage_msg;
  Compressedimage_msg.header.frame_id = id;
  Compressedimage_msg.header.stamp.nanosec = t2;
  Compressedimage_msg.header.stamp.sec = t1;
  Compressedimage_msg.set__format(msg->format);
  Compressedimage_msg.data.resize(msg->data.size());
  memcpy(Compressedimage_msg.data.data(), msg->data.data(), msg->data.size());
  // CompressedImageBack_pub_->publish(Compressedimage_msg);
}
void Convert::CompressedImage_mqtt_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  imageFront_count++;

  struct timespec t = {0, 0};
  clock_gettime(CLOCK_REALTIME, &t);
  int t2 = t.tv_nsec;
  int t1 = t.tv_sec;

  sensors_msg::ImageProto msg_pb;

  std::string str1 = msg->header.frame_id;
  std::string str2 = std::to_string(imageFront_count);
  std::string str3 = " ";
  std::string id = str1 + "" + str3 + "" + str2;

  msg_pb.set_header_frame_id(id);
  msg_pb.set_header_stamp_nanosec(t2);
  msg_pb.set_header_stamp_sec(t1);
  msg_pb.set_format(msg->format);
  msg_pb.set_data(msg->data.data(), msg->data.size());

  // base64
  // std::string base64_data = base64_encode(reinterpret_cast<const unsigned char *>(msg_pb.data().data()), msg_pb.data().size());

  std::string ser_msg;
  msg_pb.SerializeToString(&ser_msg);
  // std::cout << "format:" << msg->format.c_str() << std::endl;

  // std::string compressed_data;
  // compress_data(ser_msg, compressed_data);
  // std::cout << "ccccompressed_size:" << compressed_data.size() << std::endl;

  sensor_msgs::msg::CompressedImage imgc;
  imgc.format = msg->format;
  imgc.header.frame_id = id;
  imgc.header.stamp.nanosec = t2;
  imgc.header.stamp.sec = t1;
  imgc.data.resize(msg->data.size());
  memcpy(imgc.data.data(), msg->data.data(), msg->data.size());

  // mqtt_image_pub.pub("mqtt_image", ser_msg, 0);
  // CompressedImageBack_pub_->publish(imgc);
}

void Convert::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  imucount++;
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

  mqtt_imu_pub.pub("mqtt_imu", ser_msg, 0);

  sensor_msgs::msg::Imu RosImu;
  RosImu.set__angular_velocity(msg->angular_velocity);
  RosImu.set__angular_velocity_covariance(msg->angular_velocity_covariance);
  RosImu.header.frame_id=id;
  RosImu.header.stamp.sec=t1;
  RosImu.header.stamp.nanosec=t2;
  RosImu.set__linear_acceleration(msg->linear_acceleration);
  RosImu.set__linear_acceleration_covariance(msg->linear_acceleration_covariance);
  RosImu.set__orientation(msg->orientation);
  RosImu.set__orientation_covariance(msg->orientation_covariance);

  // imu_pub_->publish(RosImu);
}

void Convert::compress_data(const std::string &input, std::string &output)
{
  const int BUFFER_SIZE = 1024;
  char buffer[BUFFER_SIZE];

  z_stream zs;
  memset(&zs, 0, sizeof(zs));

  if (deflateInit(&zs, Z_DEFAULT_COMPRESSION) != Z_OK)
  {
    throw(std::runtime_error("deflateInit failed while compressing."));
  }

  zs.next_in = reinterpret_cast<Bytef *>(const_cast<char *>(input.data()));
  zs.avail_in = input.size();

  int ret;
  do
  {
    zs.next_out = reinterpret_cast<Bytef *>(buffer);
    zs.avail_out = BUFFER_SIZE;

    ret = deflate(&zs, Z_FINISH);
    if (output.size() < zs.total_out)
    {
      output.insert(output.end(), &buffer[0], &buffer[0] + (zs.total_out - output.size()));
    }
  } while (ret == Z_OK);

  deflateEnd(&zs);

  if (ret != Z_STREAM_END)
  {
    throw(std::runtime_error("Exception during zlib compression."));
  }
}

std::string Convert::base64_encode(const unsigned char *data, size_t length)
{
  const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  std::string encoded;
  encoded.reserve(((length + 2) / 3) * 4);

  for (size_t i = 0; i < length; i += 3)
  {
    unsigned char b1 = data[i];
    unsigned char b2 = (i + 1 < length) ? data[i + 1] : 0;
    unsigned char b3 = (i + 2 < length) ? data[i + 2] : 0;

    unsigned char b1_ = b1 >> 2;
    unsigned char b2_ = ((b1 & 0x03) << 4) | (b2 >> 4);
    unsigned char b3_ = ((b2 & 0x0F) << 2) | (b3 >> 6);
    unsigned char b4_ = b3 & 0x3F;

    encoded.push_back(base64_chars[b1_]);
    encoded.push_back(base64_chars[b2_]);
    encoded.push_back(base64_chars[b3_]);
    encoded.push_back(base64_chars[b4_]);
  }

  if (length % 3 == 1)
  {
    encoded[encoded.size() - 1] = '=';
    encoded[encoded.size() - 2] = '=';
  }
  else if (length % 3 == 2)
  {
    encoded[encoded.size() - 1] = '=';
  }

  return encoded;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rcutils_logging_initialize();
  rclcpp::spin(std::make_shared<Convert>());
  rclcpp::shutdown();
  return 0;
}