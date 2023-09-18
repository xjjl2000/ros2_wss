#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rosmessage/imagemsg.hpp"
#include "rosmessage/imumsg.hpp"
#include "rosmessage/pointcloud2msg.hpp"
#include "rosmessage/cmdvelmsg.hpp"
#include "rosmessage/location.hpp"
#include "rosmessage/pushButton.hpp"


#include <QGuiApplication>
#include "rosmessage/MainNode.hpp"
#include "QApplication"

int main(int argc, char *argv[])
{
   

  QApplication app(argc, argv);

  rclcpp::init(argc, argv);
  app.processEvents();
  rcutils_logging_initialize();

  // //https://zhuanlan.zhihu.com/p/355083258

  // auto imageMsg_ptr=std::make_shared<ImageMsg>();
  // auto imuMsg_ptr=std::make_shared<ImuMsg>();
  auto  main_node=std::make_shared<MainNode>("mainnode");
  main_node->init();
  // std::shared_ptr<rclcpp::Node> pointCloudMsg_ptr=std::make_shared<PointCloudMsg>();
  // auto cmdVel_ptr=std::make_shared<CmdVel>();
  // auto locationMsg_ptr=std::make_shared<LocationMsg>();
  // auto pushButton_ptr=std::make_shared<PushButton>();

 

  while (rclcpp::ok())
  {
    // rclcpp::spin_some(imageMsg_ptr);
    // rclcpp::spin_some(imuMsg_ptr);
    // rclcpp::spin_some(pointCloudMsg_ptr);
    // rclcpp::spin_some(cmdVel_ptr);
    // rclcpp::spin_some(locationMsg_ptr);
    rclcpp::spin_some(main_node);

    app.processEvents();

  }

  // while (rclcpp::ok())
  // {

  //   //rclcpp::spin(std::make_shared<ImageMsg>());
  //   //rclcpp::spin(std::make_shared<ImuMsg>());
  //   // rclcpp::spin(std::make_shared<PointCloudMsg>());
  //   // rclcpp::spin(std::make_shared<CmdVel>());
  //   // rclcpp::spin(std::make_shared<LocationMsg>());
  //   // rclcpp::spin(std::make_shared<PushButton>());

  //   app.processEvents();

  // }

  rclcpp::shutdown();
  // return app.exec();
}