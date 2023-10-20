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
#include "mqttClient/MqttMsgSub.hpp"
#include "QApplication"

int main(int argc, char *argv[])
{

  QApplication app(argc, argv);

  rclcpp::init(argc, argv);
  app.processEvents();
  rcutils_logging_initialize();

  // //https://zhuanlan.zhihu.com/p/355083258

  auto main_node = std::make_shared<MainNode>("mainnode");
  // main_node->init();

  MqttSubscriber mqttsubscriber;
  mqttsubscriber.init();

  while (rclcpp::ok())
  {
    rclcpp::spin_some(main_node);

    app.processEvents();
  }

  rclcpp::shutdown();
}