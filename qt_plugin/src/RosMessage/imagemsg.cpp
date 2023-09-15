#include "rosmessage/imagemsg.hpp"

ImageMsg::ImageMsg() : Node("imagemsg"), count_(0)
{
  rclcpp::QoS qos(10);
  qos = qos.best_effort();
  qos = qos.keep_last(10);

  sub_ACompressed = this->create_subscription<sensor_msgs::msg::CompressedImage>("/d455_camera/color/image_raw/compressed_A", qos,
                                                                                 std::bind(&ImageMsg::A_f_imagecallback, this, std::placeholders::_1));
  sub_BCompressed = this->create_subscription<sensor_msgs::msg::CompressedImage>("/d455_camera_nx/color/image_raw/compressed_B", qos,
                                                                                 std::bind(&ImageMsg::B_f_imagecallback, this, std::placeholders::_1));

  // 后置摄像头compressed
  sub_ARCompressed = this->create_subscription<sensor_msgs::msg::CompressedImage>("/rear_camera/camera/image_raw/compresseddddddd", qos, std::bind(&ImageMsg::A_r_compressed_imagecallback, this, std::placeholders::_1));
  sub_BRCompressed = this->create_subscription<sensor_msgs::msg::CompressedImage>("/rear_camera_nx/camera/image_raw/compressed_B", qos, std::bind(&ImageMsg::B_r_compressed_imagecallback, this, std::placeholders::_1));

  // 订阅虚拟小车image_raw
  image_raw_V = this->create_subscription<sensor_msgs::msg::Image>("/isaac_camera/camera", qos, std::bind(&ImageMsg::V_r_image_raw_callback, this, std::placeholders::_1));
}

void ImageMsg::A_f_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{

  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgCallback;
    imgCallback = cv_ptr_compressed->image;
    QImage img = convertToQImage(imgCallback);
    std::cout << "677:set\n";

    // ui_->label_image->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image->width(),ui_->label_image->height()));
    // cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  this->image_At_f.sec = msg->header.stamp.sec;
  this->image_At_f.nanosec = msg->header.stamp.nanosec;

  if (this->image_At_f.nanosec)
  {
    struct timespec image_front_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &image_front_t);
    if (this->lastTime.tv_sec != 0)
    {
      double hz = 1 / ((image_front_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (image_front_t.tv_sec - this->lastTime.tv_sec));

      // this->ui_->imageFrontHz_label->setText(QString::number(hz,'1',1));
    }
    this->lastTime = image_front_t;
    int t0 = image_front_t.tv_sec;
    int t1 = image_front_t.tv_nsec;
    double image_front_delayt = (t0 + t1 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    // this->ui_->imageFrontTime_label->setText(QString::number(image_front_delayt,'f',3));
    std::ofstream outfile("compress.txt", std::ios::app); // 创建一个名为.txt 的输出文件流
    outfile << "sec：" << msg->header.stamp.sec << "    nanosec：" << msg->header.stamp.nanosec << "    image_back_delayt:" << image_front_delayt << std::endl;
  }
}

void ImageMsg::B_f_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgCallback;
    imgCallback = cv_ptr_compressed->image;
    QImage img = convertToQImage(imgCallback);
    std::cout << "716:set\n";

    // ui_->label_image_front_B->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_front_B->width(),ui_->label_image_front_B->height()));
    // cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  this->image_Bt_f.sec = msg->header.stamp.sec;
  this->image_Bt_f.nanosec = msg->header.stamp.nanosec;
  if (this->image_Bt_f.nanosec)
  {
    struct timespec image_front_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &image_front_t);
    if (this->lastTime.tv_sec != 0)
    {
      double hz = 1 / ((image_front_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (image_front_t.tv_sec - this->lastTime.tv_sec));
      // this->ui_->imageFrontHz_label_B->setText(QString::number(hz,'1',1));
    }
    this->lastTime = image_front_t;
    int t0 = image_front_t.tv_sec;
    int t1 = image_front_t.tv_nsec;
    double image_front_delayt_B = (t0 + t1 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    // this->ui_->imageFrontTime_label_B->setText(QString::number(image_front_delayt_B,'f',3));
  }
}

void ImageMsg::A_r_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgCallback;
    if (cv_ptr_compressed == nullptr)
      throw std::runtime_error("765 nullptr");
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

  // std::cout<<"解析图片的时间为："<<msg2image_t<<std::endl;
  this->image_At_b.sec = msg->header.stamp.sec;
  this->image_At_b.nanosec = msg->header.stamp.nanosec;

  // delay

  // hz
  if (this->image_At_b.nanosec)
  {
    struct timespec image_back_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &image_back_t);
    if (this->lastTime1.tv_sec != 0)
    {
      double hz = 1 / ((image_back_t.tv_nsec - this->lastTime1.tv_nsec) * 1e-9 + (image_back_t.tv_sec - this->lastTime1.tv_sec));
      // this->ui_->imageBackHz_label->setText(QString::number(hz,'1',1));
    }
    this->lastTime1 = image_back_t;

    int t0 = image_back_t.tv_sec;
    int t1 = image_back_t.tv_nsec;
    double image_back_delayt = (t0 + t1 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    // this->ui_->imageBackTime_label->setText(QString::number(image_back_delayt,'f',3));
  }
}
void ImageMsg::B_r_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgCallback;
    imgCallback = cv_ptr_compressed->image;
    QImage img = convertToQImage(imgCallback);
    // ui_->label_image_back_B->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back_B->width(),ui_->label_image_back_B->height()));
    // cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  // std::cout<<"解析图片的时间为："<<msg2image_t<<std::endl;
  this->image_Bt_b.sec = msg->header.stamp.sec;
  this->image_Bt_b.nanosec = msg->header.stamp.nanosec;

  // delay

  // hz
  if (this->image_Bt_b.nanosec)
  {
    struct timespec image_back_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &image_back_t);
    if (this->lastTime.tv_sec != 0)
    {
      double hz = 1 / ((image_back_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (image_back_t.tv_sec - this->lastTime.tv_sec));

      // this->ui_->imageBackHz_label_B->setText(QString::number(hz,'1',1));
    }
    this->lastTime = image_back_t;

    int t0 = image_back_t.tv_sec;
    int t1 = image_back_t.tv_nsec;
    double image_back_delayt = (t0 + t1 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    // this->ui_->imageBackTime_label_B->setText(QString::number(image_back_delayt,'f',3));
  }
}

void ImageMsg::V_r_image_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{

  // struct timespec t = {0, 0};
  // clock_gettime(CLOCK_REALTIME, &t);
  // int n1=t.tv_sec;
  // int n2=t.tv_nsec;
  // double image_delayt=(n1+n2 *1e-9)-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
  // std::cout<<rearimage_count<<": rawtime="<<image_delayt<<std::endl;
  // this->ui_->imageBackTime_label_v->setText(QString::number(image_delayt,'f',3));

  try
  {
    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat imgCallback;
    imgCallback = cv_ptr_compressed->image;
    QImage img = convertToQImage(imgCallback);
    // ui_->label_image_back_virtual->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back_virtual->width(),ui_->label_image_back_virtual->height()));
    // cv::waitKey(1);
  }
  catch (cv_bridge::Exception &e)
  {
    // ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  this->image_Bt_b.sec = msg->header.stamp.sec;
  this->image_Bt_b.nanosec = msg->header.stamp.nanosec;
  if (this->image_Bt_b.nanosec)
  {
    struct timespec image_back_t = {0, 0};
    clock_gettime(CLOCK_REALTIME, &image_back_t);
    double time = (image_back_t.tv_nsec - this->lastTime.tv_nsec) * 1e-9 + (image_back_t.tv_sec - this->lastTime.tv_sec);
    if (this->lastTime.tv_sec != 0)
    {
      double hz = 1 / time;
      // this->ui_->imageBackHz_label_v->setText(QString::number(hz,'1',1));
    }
    this->lastTime = image_back_t;
    int n1 = image_back_t.tv_sec;
    int n2 = image_back_t.tv_nsec;
    double image_delayt = (n1 + n2 * 1e-9) - (msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9);
    // this->ui_->imageBackTime_label_v->setText(QString::number(image_delayt,'f',3));
  }
}

QImage ImageMsg::convertToQImage(const cv::Mat &image)
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