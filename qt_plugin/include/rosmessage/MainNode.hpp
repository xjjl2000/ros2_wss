#ifndef MAINNODE_HPP
#define MAINNODE_HPP


#include "rclcpp/rclcpp.hpp"
#include<vector>
#include "src/rosserver/RosPointCloud2Callback.cpp"
#include "src/rosserver/RosImageCallback.cpp"
#include "src/rosserver/RosLocationCallback.cpp"
#include "src/rosserver/RosImuCallback.cpp"
#include "src/rosserver/RosWheelSpeedCallback.cpp"
#include "src/rosserver/RosCmdVelCallback.cpp"


class MainNode : public rclcpp::Node
{
        public:
        MainNode(const std::string);

        void init();

        private:
                std::shared_ptr<RosPointCloud2Callback> RosPointCloud2;
                std::shared_ptr<RosImageFrontCallback> RosImageFront_ptr;
                std::shared_ptr<RosImageBackCallback> RosImageBack_ptr;
                std::shared_ptr<RosImuCallback> RosImu_ptr;
                std::shared_ptr<RosImuMagneticFieldCallback> RosImuMagneticField_ptr;
                std::shared_ptr<RosPoseCallback> RosPose_ptr;
                std::shared_ptr<RosPose2DCallback> RosPose2D_ptr;
                std::shared_ptr<RosPoseStampedCallback> RosPoseStamped_ptr;
                std::shared_ptr<Ros4WheelDifferentialCallback> Ros4WheelDifferential_ptr;
                std::shared_ptr<Ros2WheelDifferentialCallback> Ros2WheelDifferential_ptr;
                std::shared_ptr<RosjointStateCallback> RosjointState_ptr;
                std::shared_ptr<RosCmdVelCallback> RosCmdVel_ptr;


};



#endif