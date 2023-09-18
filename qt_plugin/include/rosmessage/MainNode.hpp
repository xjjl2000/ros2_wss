#ifndef MAINNODE_HPP
#define MAINNODE_HPP


#include "rclcpp/rclcpp.hpp"
#include<vector>
#include "src/rosserver/RosPointCloud2Callback.cpp"
#include "src/rosserver/RosImageCallback.cpp"
class MainNode : public rclcpp::Node
{
        public:
        MainNode(const std::string);

        void init();

        private:
                std::shared_ptr<RosPointCloud2Callback> RosPointCloud2;
                std::shared_ptr<RosImageCallback> RosImage_ptr;

};



#endif