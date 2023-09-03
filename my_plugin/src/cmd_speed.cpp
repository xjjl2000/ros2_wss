#include "my_plugin/cmd_speed.hpp"
#include <iostream>


namespace rviz2_plugin
{


        void Cmd_Speed::cmd_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg,QLabel *label1,QLabel *label2)
        {
                 std::cout<<"222"<<std::endl;

                label1->setText(QString::number(msg->linear.x,'f',2));
                label2->setText(QString::number(msg->angular.z,'f',2));
        }

}
