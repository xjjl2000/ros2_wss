#ifndef SPEED_DEF_H__
#define SPEED_DEF_H__

#include "geometry_msgs/msg/twist.hpp"
#include "qlabel.h"

namespace rviz2_plugin
{
    class Cmd_Speed 
    {
    public:

        void cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg,QLabel *label1,QLabel *label2);


    };

}  // namespace rviz2_plugin

#endif