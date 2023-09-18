#include "rosmessage/MainNode.hpp"


MainNode::MainNode(const std::string name ) :  Node(name)
{

}

void MainNode::init()
{
  rclcpp::QoS qos(10);
  qos = qos.best_effort();
  qos = qos.keep_last(10);

  RosPointCloud2=std::make_shared< RosPointCloud2Callback>(std::string("/scan"),this);
  RosPointCloud2->sub(10);

  
  RosImage_ptr=std::make_shared<RosImageCallback>(std::string("/rear_camera/camera/image_raw/compresseddddddd"),this);
  RosImage_ptr->sub(qos);

  RosImu_ptr=std::make_shared<RosImuCallback>(std::string("/imu"),this);
  RosImu_ptr->sub(qos);

  RosImuMagneticField_ptr=std::make_shared<RosImuMagneticFieldCallback>(std::string("/imu/mag"),this);
  RosImuMagneticField_ptr->sub(qos);

  RosPose_ptr=std::make_shared<RosPoseCallback>(std::string("/location_2Ddddd"),this);
  RosPose_ptr->sub(qos);

  RosPose2D_ptr=std::make_shared<RosPose2DCallback>(std::string("/location_2D"),this);
  RosPose2D_ptr->sub(qos);

  RosPoseStamped_ptr=std::make_shared<RosPoseStampedCallback>(std::string("/current_pose"),this);
  RosPoseStamped_ptr->sub(qos);

  RosPoseStamped_ptr=std::make_shared<RosPoseStampedCallback>(std::string("/uwb_location"),this);
  RosPoseStamped_ptr->sub(qos);

  Ros4WheelDifferential_ptr=std::make_shared<Ros4WheelDifferentialCallback>(std::string("/odom_combined_A"),this);
  Ros4WheelDifferential_ptr->sub(qos);

  Ros2WheelDifferential_ptr=std::make_shared<Ros2WheelDifferentialCallback>(std::string("/odom_combined_B"),this);
  Ros2WheelDifferential_ptr->sub(qos);

  RosjointState_ptr=std::make_shared<RosjointStateCallback>(std::string("/joint_states_isaac"),this);
  RosjointState_ptr->sub(qos);


}
