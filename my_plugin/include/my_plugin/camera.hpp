#ifndef CAMERA_DEF_H__
#define CAMERA_DEF_H__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"


namespace rviz2_plugin
{
    class CameraModule 
    {
    public:

        void image_callback();


    };
}  // namespace rviz2_plugin

#endif