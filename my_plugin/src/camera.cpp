#include "my_plugin/rviz_push_button.hpp"
#include "my_plugin/camera.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"


namespace rviz2_plugin
{


        void CameraModule::image_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg,QLabel *label1,QLabel *label2)
        {
                label1->setText(QString::number(msg->linear.x,'f',2));
                label2->setText(QString::number(msg->angular.z,'f',2));
        }

}