#include "rosmessage/MainNode.hpp"


MainNode::MainNode(const std::string name ) :  Node(name)
{

}

void MainNode::init()
{

  RosPointCloud2=std::make_shared< RosPointCloud2Callback>(std::string("/scan"),this);
  RosPointCloud2->sub(10);

  rclcpp::QoS qos(10);
  qos = qos.best_effort();
  qos = qos.keep_last(10);
  RosImage_ptr=std::make_shared<RosImageCallback>(std::string("/rear_camera/camera/image_raw/compresseddddddd"),this);
  RosImage_ptr->sub(qos);


}
