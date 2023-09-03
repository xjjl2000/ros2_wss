#include<functional>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "geometry_msgs.pb.h"
#include "sensors_msg.pb.h"
#include "pointcloud2.pb.h"

#include "factoryCallback.hpp"



namespace callbackFactory{
        int cmdVelCallback(void *context, char *topicName, int topicLen, MQTTClient_message *message){
                geometry_msgs::Twist twist_msg;
                twist_msg.ParseFromArray(message->payload, message->payloadlen);

                // std::cout<<"_receivedmsg"<<Mqtt_sub::_receivedmsg.c_str()<< "  size"<<Mqtt_sub::_receivedmsg.size()<<std::endl;
                std::cout << "FROM topic: " << topicName << "  msg:" << (char *)message->payload << std::endl;
                std::cout << "twist_msg: linear_x" << twist_msg.linear_x() << "  linear_y " << twist_msg.linear_y() << " size:" << twist_msg.ByteSize() << std::endl;

                // message.ParseFromArray((const char *)msg->payload, msg->payloadlen);
                // std::cout << "GOT message:\n" << message.DebugString();
                MQTTClient_freeMessage(&message);
                MQTTClient_free(topicName);
                return 1;

        }
        
        int mqttpublishCallback(void *context, char *topicName, int topicLen, MQTTClient_message *message){
                //printf("\nmqttpublish start\n");
                //payloadptr = message->payload;
                std::cout << "FROM topic: " << topicName << "  msg_size:" << message->payloadlen << std::endl;
                // message.ParseFromArray((const char *)msg->payload, msg->payloadlen);
                // std::cout << "GOT message:\n" << message.DebugString();
                MQTTClient_freeMessage(&message);
                MQTTClient_free(topicName);
                return 1;
        }


        int imgCallback(void *context, char *topicName, int topicLen, MQTTClient_message *message){
                //todo something
                // this define
                std::cout<<"FROM topic: " << topicName <<"  img_size:"<<message->payloadlen<<std::endl;
                sensors_msg::ImageProto msg_pb;
                msg_pb.ParseFromString((const char*)message->payload);
                std::cout<<"msg_pb_size:"<<msg_pb.ByteSize()<<std::endl;
                sensor_msgs::msg::CompressedImage imgc;
                imgc.set__format(msg_pb.format());
                imgc.header.frame_id=msg_pb.header_frame_id();
                imgc.header.stamp.nanosec=msg_pb.header_stamp_nanosec();
                imgc.header.stamp.sec=msg_pb.header_stamp_sec();
                imgc.data.resize(msg_pb.data().size());
                memcpy(imgc.data.data(),msg_pb.data().data(),msg_pb.data().size());

        //         try
        //         {
        //             cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(imgc,sensor_msgs::image_encodings::BGR8);
        //             cv::Mat imgCallback;
        //             imgCallback = cv_ptr_compressed->image;
        //             QImage img = convertToQImage(imgCallback);
        //             ui_->label_image_back->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back->width(),ui_->label_image_back->height()));
                   
        //         }
        //     catch (cv_bridge::Exception& e)
        //         {
        //         //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        //         }



                MQTTClient_freeMessage(&message);
                MQTTClient_free(topicName);
                return 1;
        }

        int pointcloud2Callback(void *context, char *topicName, int topicLen, MQTTClient_message *message){

        }

        QImage convertToQImage(const cv::Mat& image) {
                cv::Mat rgb;
                if (image.channels() == 3) {
                // cvt Mat BGR 2 QImage RGB
                cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
                } else {
                rgb = image.clone();
                }

                QImage img(rgb.cols, rgb.rows, QImage::Format_RGB888);
                uchar* imageData = img.bits();  // 获取图像数据指针

                // 将像素数据复制到QImage中
                memcpy(imageData, rgb.data, rgb.cols * rgb.rows * 3);  // 假设每个像素有三个通道

                return img;
        }
        

}


callbackFunction factoryCallback(const std::string &topic)
{
        // printf("123456%s\n",topic.c_str());
        //this return and call
        
        if (topic == "mqtt_cmd_vel")
                return callbackFactory::cmdVelCallback;
        else if(topic == "mqttpublish")
                return callbackFactory::mqttpublishCallback;
        else if(topic == "mqttpublish_img")
                return callbackFactory::imgCallback;
        else if(topic == "mqtt_pointcloud2")
                return callbackFactory::pointcloud2Callback;
        else{
                //throw std::runtime_error("can't match pattern,will return nullptr\n");
                printf("can't match pattern,will return call a default function.\n");
                return callbackFactory::mqttpublishCallback;
        }
                
        
}
