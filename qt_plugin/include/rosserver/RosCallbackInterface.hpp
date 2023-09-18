#ifndef ROSCALLBACKINTERFACE_DEF_H__
#define ROSCALLBACKINTERFACE_DEF_H__

#include "rclcpp/rclcpp.hpp"


template<
  typename MessageT,
  typename CallbackT>

class RosCallbackInterface
{
        private:
        virtual void callback(const CallbackT msg)=0;
        typename rclcpp::Subscription<MessageT>::SharedPtr subPtr;
        std::string topic;
        rclcpp::Node* node;


        public:
        RosCallbackInterface(const std::string topic,rclcpp::Node* node):topic(topic),node(node){}

        void sub(const rclcpp::QoS & qos){
                subPtr = node->create_subscription<MessageT>
                        (topic, qos, std::bind(&RosCallbackInterface::callback, this, std::placeholders::_1));

        };



};
#endif