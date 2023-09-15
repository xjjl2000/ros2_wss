#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "rosmessage/imagemsg.hpp"
#include "rosmessage/imumsg.hpp"
#include "rosmessage/pointcloud2msg.hpp"
#include "rosmessage/cmdvelmsg.hpp"
#include "rosmessage/location.hpp"
#include "rosmessage/pushButton.hpp"
#include <QGuiApplication> 
#include "QApplication"

int main(int argc, char *argv[])
{
  
  QApplication app(argc, argv);
  
  rclcpp::init(argc, argv);
  app.processEvents();
  rcutils_logging_initialize();
  while (rclcpp::ok())
  {
    rclcpp::spin(std::make_shared<ImageMsg>());
    rclcpp::spin(std::make_shared<ImuMsg>());
    rclcpp::spin(std::make_shared<PointCloudMsg>());
    rclcpp::spin(std::make_shared<CmdVel>());
    rclcpp::spin(std::make_shared<LocationMsg>());
    rclcpp::spin(std::make_shared<PushButton>());

    app.processEvents();

  }
  
  

  rclcpp::shutdown();
  // return app.exec();
}