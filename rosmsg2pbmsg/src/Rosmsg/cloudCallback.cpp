#include "Rosmsg/cloudCallback.hpp"

PointCloudMsg::PointCloudMsg() : Node("cloud_convert")
{
        rclcpp::QoS qos(10);
        qos.best_effort();
        qos.keep_last(10);

        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10, std::bind(&PointCloudMsg::pointcloud2_sub_callback, this, std::placeholders::_1));

        laserScan_sub = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos, std::bind(&PointCloudMsg::laserScanCallback, this, std::placeholders::_1));
        laserScan_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan/PointCloudMsg", 10);

        
}

void PointCloudMsg::pointcloud2_sub_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
        {
                // 获取当前时间
                rclcpp::Time now = this->get_clock()->now();
                builtin_interfaces::msg::Time tt = now;
                int n1 = tt.sec;
                int n2 = tt.nanosec;

                double pC2_pb_delayt = (n1 + n2 * 1e-9) - (cloud->header.stamp.sec + cloud->header.stamp.nanosec * 1e-9);

                struct timespec t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &t);
                int t2 = t.tv_nsec;
                int t1 = t.tv_sec;
                pointcloud2::pointCloud2_pb pbmsg;
                pbmsg.set_frame_id(cloud->header.frame_id);
                pbmsg.set_height(cloud->height);
                pbmsg.set_width(cloud->width);
                pbmsg.set_point_step(cloud->point_step);
                pbmsg.set_row_step(cloud->row_step);
                pbmsg.set_stamp_nanosec(t2);
                pbmsg.set_stamp_sec(t1);
                pbmsg.set_is_bigendian(false);
                pbmsg.set_is_dense(true);
                // pbmsg.set_PointCloudMsg_time(pC2_pb_delayt);
                pbmsg.set_stamp_nanosec(t2);
                pbmsg.set_stamp_sec(t1);
                pbmsg.mutable_data()->resize(cloud->width * cloud->point_step);
                memcpy(const_cast<void *>(static_cast<const void *>(pbmsg.data().data())), cloud->data.data(), cloud->width * cloud->point_step);

                const auto ser_start_t = std::chrono::high_resolution_clock::now();
                std::string ser_msg = pbmsg.SerializeAsString();
                const auto ser_end_t = std::chrono::high_resolution_clock::now();
                const auto ser_t = std::chrono::duration_cast<std::chrono::nanoseconds>(ser_end_t - ser_start_t).count();

                // mqtt_cloud_pub.pub("mqtt_pointcloud2", ser_msg, 0);

                // pub_->publish(buff_msg);
        }

        void PointCloudMsg::laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
        {
                laserscan_count++;
                std::cout<<"laserscan count:"<<laserscan_count<<std::endl;

                struct timespec t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &t);
                int t2 = t.tv_nsec;
                int t1 = t.tv_sec;

                sensor_msgs::msg::LaserScan laserScanmsg;

                std::string str1 = msg->header.frame_id;
                std::string str2 = std::to_string(laserscan_count);
                std::string str3 = " ";
                std::string id = str1 + "" + str3 + "" + str2;

                laserScanmsg.header.set__frame_id(id);
                laserScanmsg.header.stamp.nanosec = t2;
                laserScanmsg.header.stamp.sec = t1;
                laserScanmsg.set__angle_increment(msg->angle_increment);
                laserScanmsg.set__angle_max(msg->angle_max);
                laserScanmsg.set__angle_min(msg->angle_min);
                laserScanmsg.set__intensities(msg->intensities);
                laserScanmsg.set__range_max(msg->range_max);
                laserScanmsg.set__range_min(msg->range_min);
                laserScanmsg.set__ranges(msg->ranges);
                laserScanmsg.set__scan_time(msg->scan_time);
                laserScanmsg.set__time_increment(msg->time_increment);

                laserScan_pub->publish(laserScanmsg);
        }
