#include "mqttClient/MqttMsgSub.hpp"

MqttSubscriber::MqttSubscriber()
{

}

void MqttSubscriber::init()
{

        mqtt::connect_options sub_conn_opts;
        sub_conn_opts.set_clean_session(true);

        auto actionCallbackPtr = std::make_shared<CompentsActionCallback>();
        auto cac_ptr = actionCallbackPtr.get();
        std::string sub_name = "ActionCallback";
        cac_ptr->setFailureFunc(std::make_shared<FailureCallback>(sub_name));
        cac_ptr->setSuccessFunc(std::make_shared<SuccessCallback>(sub_name));


        auto sub_callback_ptr = std::make_shared<CompentsCallback>();
        auto sub_callback_ptr_r = sub_callback_ptr.get();
        sub_callback_ptr_r->setConnectedFunc(std::make_shared<PrintConnectedCallback>());
        sub_callback_ptr_r->setConnectionLostFunc(std::make_shared<ReConnectConnectionLostCallback>(actionCallbackPtr));
        sub_callback_ptr_r->setMessageArrivedFunc(std::make_shared<QlabelShowMessageArrivedCallback>());

        //front image
        mqttFrontImage_sub = MyMqttClient("mqtt_frontimg_sub", "192.168.31.107", 1884,
                                        sub_conn_opts,
                                        sub_callback_ptr,
                                        actionCallbackPtr);
        mqttFrontImage_sub.sub("mqtt_imageFront",0);

        //back image
        mqttBackImage_sub = MyMqttClient("mqtt_backimg_sub", "192.168.31.107", 1884,
                                        sub_conn_opts,
                                        sub_callback_ptr,
                                        actionCallbackPtr);
        mqttBackImage_sub.sub("mqtt_imageBack",0);

        //imu
        mqttImu_sub = MyMqttClient("mqtt_imu_sub", "192.168.31.107", 1884,
                                        sub_conn_opts,
                                        sub_callback_ptr,
                                        actionCallbackPtr);
        mqttImu_sub.sub("mqtt_imu",0);

        //laserscan
        mqttLaserScan_sub = MyMqttClient("mqtt_scan_sub", "192.168.31.107", 1884,
                                        sub_conn_opts,
                                        sub_callback_ptr,
                                        actionCallbackPtr);
        mqttLaserScan_sub.sub("mqtt_laserscan",0);

}