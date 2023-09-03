#include "my_plugin/rviz_push_button.hpp"

#include <pluginlib/class_list_macros.hpp>
#include "rviz_common/display.hpp"
#include <rviz_common/panel.hpp>
#include <class_loader/class_loader.hpp>

#include <QStringList>
#include <QBoxLayout>
#include <cv_bridge/cv_bridge.h>
#include <QObject>
#include <QComboBox>

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.hpp>
#include "image_transport/subscriber.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/twist.hpp"//Twist消息的结构，其中linear 的x就是代表前进方向的速度，单位为m/s。
                                      // angular 的z就代表机器人的绕中心旋转的角速度，单位为 弧度/s (rad/s)
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QChartView>
#include <QPaintEvent> //用于绘画事件
#include <QtCore> 
#include <sys/time.h>
#include <qdatetime.h>

#include "sensors_msg.pb.h"
#include "pointcloud2.pb.h"
#include "geometry_msgs.pb.h"
#include "google/protobuf/util/time_util.h"
#include "std_msgs/msg/byte_multi_array.hpp"
#include "zlib.h"
#include <fstream>
#include <iostream>
#include "rmw/qos_profiles.h"


#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "mosquitto.h"

#define DEST_PORT 8000
#define DSET_IP_ADDRESS  "192.168.1.131"
#define SERV_PORT   8000
#define BUFFER_SIZE 370000
#define MAX_PACKET_SIZE 1400



PLUGINLIB_EXPORT_CLASS(rviz2_plugin::VR_interaLsys, rviz_common::Panel)

namespace rviz2_plugin
{

    VR_interaLsys::VR_interaLsys(QWidget *parent)
        : Panel(parent), ui_(new Ui::myWidget())
    {
        ui_->setupUi(this);
        ui_->location_label->installEventFilter(this);//过滤器
        ui_->location_label_B->installEventFilter(this);//过滤器

        
        QString imagePath = "src/my_plugin/resource/wheel.jpeg";  // 图片路径
        QPixmap pixmap(imagePath);  // 创建一个QPixmap对象
        ui_->wheel->setPixmap(pixmap.scaled(ui_->wheel->width(),ui_->wheel->height()));  // 在部件上设置背景图片
        ui_->wheel_B->setPixmap(pixmap.scaled(ui_->wheel->width(),ui_->wheel->height())); 
        ui_->wheel_V->setPixmap(pixmap.scaled(ui_->wheel->width(),ui_->wheel->height()));

        //set timer
        // m_timer = new QTimer(this);已修改为下CurrentTime
        // m_timer->setSingleShot(false);
        // m_timer->start(200);
        //Chart set       
        A_tabWidget=new QTabWidget();
        B_tabWidget=new QTabWidget();      

        
        set_tabWidget_test(B_tabWidget,Acc_B,Angual_B,Mag_B,ui_->chart_label_v);
        set_tabWidget_test(A_tabWidget,Acc,Angual,Mag,ui_->chart_label);//test


        CurrentTime = new QTimer(this);
        CurrentTime->start(200);
        connect(CurrentTime, SIGNAL(timeout()), this, SLOT(slotTimeout_test_B()));  
        connect(CurrentTime, SIGNAL(timeout()), this, SLOT(slotTimeout_test()));        


        connect(CurrentTime,&QTimer::timeout,[=]()
        {
            //时钟
            QDateTime current_time = QDateTime::currentDateTime(); 
            QString StrCurrentTime = current_time.toString("yyyy-MM-dd hh:mm:ss.zzz ddd"); 
            ui_->dateTime_label->setText(StrCurrentTime);
            QFont font2("color:rgb(50,205,50);font:bold;font-size:15px");
            ui_->dateTime_label->setFont(font2);

        
        });
    }

    VR_interaLsys::~VR_interaLsys(){
        delete ui_;
    };

    void VR_interaLsys::onInitialize()
    {

        
        auto intra_comms_options = rclcpp::NodeOptions{}.use_intra_process_comms(true);

        // Mqtt_pub t("192.168.1.131",1884);
        // t.init();
        // t.pub_message(1,"6666");
        // t.publish("huati","666");
        geometry_msg::Twist cmdmsg;
        cmdmsg.set_angular_x(0);
        cmdmsg.set_angular_y(0);
        cmdmsg.set_angular_z(0);
        cmdmsg.set_linear_x(1* car_setting::RemoteForwardSpeed);
        cmdmsg.set_linear_y(2);
        cmdmsg.set_linear_z(0);
        std::string ser_msg;
        cmdmsg.SerializeToString(&ser_msg);

        
        Mqtt_pub t("192.168.2.107",1884);
        t.init();
        const int MAXLENS=1024*1024*8;
        //const int MAXLENS=20;
        std::string s(MAXLENS+1,'a'); 
        s[MAXLENS]  =0;      
        //s[10]  =0;   
        // t.pub_message("huati","huati_test");
        //t.pub_message("mqttpublish","hello,world!");
        // t.pub_message("mqttpublish",s);
        t.pub_message("cmd_vel",ser_msg);


        geometry_msg::Twist cmdmsgde;
        cmdmsgde.ParseFromArray(ser_msg.data(),ser_msg.size());
        std::cout<<"deser:"<<cmdmsgde.linear_x()<<std::endl;
        // Mqtt_sub s("192.168.1.131",1884);
        // s.init();
        // std::string image_b_mqtt=s.sub("imagemqtt");
        // sensors_msg::ImageProto msg_pb;
        // msg_pb.ParseFromString(msg_pb.data().data());
        // cv::Mat converted_image = convertToCVMat(msg_pb);
        //     // 显示图像在 UI 上
        // QImage img(converted_image.data, converted_image.cols, converted_image.rows, 
        //             converted_image.step, QImage::Format_RGB888);

        // ui_->label_image_back_virtual->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image->width(), ui_->label_image->height()));



        node_ = std::make_shared<rclcpp::Node>("myrviznode");
        par_V=node_->create_publisher<std_msgs::msg::String>("controller_parm",10);
  
        //image_sub
        //前置摄像头compressed
        rclcpp::QoS qos(10);
        qos=qos.best_effort();
        qos=qos.keep_last(10);

        image_compressed_sub_A = node_->create_subscription<sensor_msgs::msg::CompressedImage>
        ("/d455_camera/color/image_raw/compressed_A", qos, std::bind(&VR_interaLsys::A_front_imagecallback,this,std::placeholders::_1));
        image_compressed_sub_B = node_->create_subscription<sensor_msgs::msg::CompressedImage>
        ("/d455_camera_nx/color/image_raw/compressed_B", qos, std::bind(&VR_interaLsys::B_front_imagecallback,this,std::placeholders::_1));

        Compressedimage_f_pb_A= node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/camera_pb_Af", qos, std::bind(&VR_interaLsys::Af_ser_compressedimagecallback,this,std::placeholders::_1));

        //后置摄像头compressed
        // const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();      
        image_back_compressed_sub_A = node_->create_subscription<sensor_msgs::msg::CompressedImage>
        ("/rear_camera/camera/image_raw/compressed_AA", qos, std::bind(&VR_interaLsys::A_back_compressed_imagecallback,this,std::placeholders::_1));
        image_back_compressed_sub_B = node_->create_subscription<sensor_msgs::msg::CompressedImage>
        ("/rear_camera_nx/camera/image_raw/compressed_B", qos, std::bind(&VR_interaLsys::B_back_compressed_imagecallback,this,std::placeholders::_1));

        //订阅image_raw
        image_raw_V=node_->create_subscription<sensor_msgs::msg::Image>
        ("/isaac_camera/camera",qos,std::bind(&VR_interaLsys::V_back_image_raw_callback,this,std::placeholders::_1));

        //后置摄像头----image_raw添加protobuf反序列化
        image_back_pb_V = node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/rear_camera_pb_Ar", qos, std::bind(&VR_interaLsys::V_ser_imagecallback,this,std::placeholders::_1));

        image_f_pb_A=node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/rear_camera_pb_Aff", qos, std::bind(&VR_interaLsys::Af_ser_imagecallback,this,std::placeholders::_1));
        image_f_pb_B=node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/rear_camera_pb_Arr", qos, std::bind(&VR_interaLsys::Ar_ser_imagecallback,this,std::placeholders::_1));
        image_r_pb_A=node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/rear_camera_pb_Bf", qos, std::bind(&VR_interaLsys::Bf_ser_imagecallback,this,std::placeholders::_1));
        image_r_pb_B=node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/rear_camera_pb_Br", qos, std::bind(&VR_interaLsys::Br_ser_imagecallback,this,std::placeholders::_1));

        //remote_control
        remote_control_pub_A=node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
        remote_control_pub_B=node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_B",10);
        remote_control_pub_V=node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_V",10);


        //data_show
        //uwb-学校
        // point_uwb=node_->create_subscription<geometry_msgs::msg::Pose>("/uwb_locationss",qos,std::bind(&VR_interaLsys::uwb_poseA_callback,this,std::placeholders::_1));
        point_ubw_B=node_->create_subscription<geometry_msgs::msg::Pose2D>("/location_2D",qos,std::bind(&VR_interaLsys::uwb_poseB_callback,this,std::placeholders::_1));

        //april_tag-29
        point=node_->create_subscription<geometry_msgs::msg::PoseStamped>("/current_pose",qos,std::bind(&VR_interaLsys::poseA_callback,this,std::placeholders::_1));
        point_B=node_->create_subscription<geometry_msgs::msg::PoseStamped>("/uwb_location",qos,std::bind(&VR_interaLsys::poseB_callback,this,std::placeholders::_1));

        jointstate_V=node_->create_subscription<sensor_msgs::msg::JointState>("/joint_states_isaac",10,std::bind(&VR_interaLsys::jointState_callback,this,std::placeholders::_1));
        nmp_A=node_->create_subscription<nav_msgs::msg::Odometry>("/odom_combined_A",10,std::bind(&VR_interaLsys::speedA,this,std::placeholders::_1));
        nmp_B=node_->create_subscription<nav_msgs::msg::Odometry>("/odom_combined_B",10,std::bind(&VR_interaLsys::speedB,this,std::placeholders::_1));

        cmd_vel_A=node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[this](const geometry_msgs::msg::Twist::ConstSharedPtr msg){
            cmd_.cmd_callback(msg,ui_->actual_lineal_speed,ui_->actual_angual_speed);
        });
        cmd_vel_B=node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_B",10,[this](const geometry_msgs::msg::Twist::ConstSharedPtr msg){
            cmd_.cmd_callback(msg,ui_->actual_lineal_speed_B,ui_->actual_angual_speed_B);
        });
        cmd_vel_V=node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_V",10,[this](const geometry_msgs::msg::Twist::ConstSharedPtr msg){
            cmd_.cmd_callback(msg,ui_->actual_lineal_speed_V,ui_->actual_angual_speed_V);           
        });

        //IMU
        actualDataLine_sub_A=node_->create_subscription<sensor_msgs::msg::Imu>
        ("/imu/data_A",10,std::bind(&VR_interaLsys::imudata_callback,this,std::placeholders::_1));
        actualMag_data_sub_A=node_->create_subscription<sensor_msgs::msg::MagneticField>
        ("/imu/mag",10,std::bind(&VR_interaLsys::imumag_callback,this,std::placeholders::_1));

        actualDataLine_sub_B=node_->create_subscription<sensor_msgs::msg::Imu>
        ("/imu",10,std::bind(&VR_interaLsys::imudata_callback_B,this,std::placeholders::_1));
        actualMag_data_sub_B=node_->create_subscription<sensor_msgs::msg::MagneticField>
        ("/imu/mag_B",10,std::bind(&VR_interaLsys::imumag_callback_B,this,std::placeholders::_1));

        //point_cloud2
        pointcloud2_sub_A=node_->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/livox/lidarrr",10,std::bind(&VR_interaLsys::pointcloud2_callback,this,std::placeholders::_1));
        pointcloud2pb_sub_A=node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/lidar_pb",10,std::bind(&VR_interaLsys::pointcloud2pb_callback_A,this,std::placeholders::_1));
        pointcloud2pb_sub_B=node_->create_subscription<std_msgs::msg::ByteMultiArray>
        ("/lidar_pb_B",10,std::bind(&VR_interaLsys::pointcloud2pb_callback_B,this,std::placeholders::_1));
        pb_2_cloud2_A=node_->create_publisher<sensor_msgs::msg::PointCloud2>("pb_2_Pointcloud2",10);
        pb_2_cloud2_B=node_->create_publisher<sensor_msgs::msg::PointCloud2>("pb_2_Pointcloud2_B",10);


        // VR_interaLsys::alive=true;
        // VR_interaLsys::connected=false;

        // UDP_send();

        // UDP_receive();
        

        //按钮初始化
        set_pushbutton(ui_->up_pushButton);
        set_pushbutton(ui_->down_pushButton);
        set_pushbutton(ui_->left_pushButton);
        set_pushbutton(ui_->right_pushButton);
        set_pushbutton(ui_->stop_pushButton);

        set_pushbutton(ui_->up_pushButton_V);
        set_pushbutton(ui_->down_pushButton_V);
        set_pushbutton(ui_->left_pushButton_V);
        set_pushbutton(ui_->right_pushButton_V);
        set_pushbutton(ui_->stop_pushButton_V);

        set_pushbutton(ui_->up_pushButton_B);
        set_pushbutton(ui_->down_pushButton_B);
        set_pushbutton(ui_->left_pushButton_B);
        set_pushbutton(ui_->right_pushButton_B);
        set_pushbutton(ui_->stop_pushButton_B);

        //按钮与键盘绑定
        ui_->up_pushButton->setShortcut(Qt::Key_Up);
        ui_->down_pushButton->setShortcut(Qt::Key_Down);
        ui_->left_pushButton->setShortcut(Qt::Key_Left);
        ui_->right_pushButton->setShortcut(Qt::Key_Right);
        ui_->stop_pushButton->setShortcut(Qt::Key_Space);


        // Connect with Qt Widget信号与槽函数的绑定
        connect(A_tabWidget, SIGNAL(tabCloseRequested(int)), this, SLOT(onTabCloseRequested(int)));
        connect(ui_->run_pushButton, SIGNAL(clicked()), this, SLOT(vedit_button_clicked()));
        connect(ui_->run_pushButton_B, SIGNAL(clicked()), this, SLOT(vedit_button_clicked_B()));
        connect(ui_->run_pushButton_V, SIGNAL(clicked()), this, SLOT(vedit_button_clicked_V()));

        connect(ui_->up_pushButton, SIGNAL(clicked()), this, SLOT(up_button_clicked()));
        connect(ui_->down_pushButton, SIGNAL(clicked()), this, SLOT(down_button_clicked()));
        connect(ui_->left_pushButton, SIGNAL(clicked()), this, SLOT(left_button_clicked()));
        connect(ui_->right_pushButton, SIGNAL(clicked()), this, SLOT(right_button_clicked()));
        connect(ui_->stop_pushButton, SIGNAL(clicked()), this, SLOT(stop_button_clicked()));

        connect(ui_->up_pushButton_B, SIGNAL(clicked()), this, SLOT(up_button_clicked_B()));
        connect(ui_->down_pushButton_B, SIGNAL(clicked()), this, SLOT(down_button_clicked_B()));
        connect(ui_->left_pushButton_B, SIGNAL(clicked()), this, SLOT(left_button_clicked_B()));
        connect(ui_->right_pushButton_B, SIGNAL(clicked()), this, SLOT(right_button_clicked_B()));
        connect(ui_->stop_pushButton_B, SIGNAL(clicked()), this, SLOT(stop_button_clicked_B()));

        connect(ui_->up_pushButton_V, SIGNAL(clicked()), this, SLOT(up_button_clicked_V()));
        connect(ui_->down_pushButton_V, SIGNAL(clicked()), this, SLOT(down_button_clicked_V()));
        connect(ui_->left_pushButton_V, SIGNAL(clicked()), this, SLOT(left_button_clicked_V()));
        connect(ui_->right_pushButton_V, SIGNAL(clicked()), this, SLOT(right_button_clicked_V()));
        connect(ui_->stop_pushButton_V, SIGNAL(clicked()), this, SLOT(stop_button_clicked_V()));

        connect(ui_->reset_pushButton, SIGNAL(clicked()), this, SLOT(reset_button_clicked()));
        connect(ui_->reset_pushButton_B, SIGNAL(clicked()), this, SLOT(reset_button_clicked()));
        connect(ui_->reset_pushButton_V, SIGNAL(clicked()), this, SLOT(reset_button_clicked()));

        connect(ui_->points_clear,SIGNAL(clicked()),this,SLOT(points_cleart_button_clicked()));
        connect(ui_->parm_button,SIGNAL(clicked()),this,SLOT(controller_parm_button_clicked()));



        spin_thread_ = std::thread(&VR_interaLsys::spin, this);

    }

    void VR_interaLsys::onEnable()
    {
        show();
        parentWidget()->show();
    }

    void VR_interaLsys::onDisable()
    {
        hide();
        parentWidget()->hide();
    }
    
    void VR_interaLsys::spin(){
        while (running_) {
            rclcpp::spin_some(node_);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void VR_interaLsys::paintEvent()
    {

        //printf("\npaintEvent\n");

        // 设置画笔颜色
        QPen penDegree;
        penDegree.setColor(Qt::black);
        penDegree.setWidth(2);

        // 设置画笔颜色
        QPen penDegree2;
        penDegree2.setColor(Qt::gray);
        penDegree2.setWidth(0.01);

        QPen p1;
        p1.setColor(Qt::red);
        p1.setWidth(0.5);

        QPen pB;
        pB.setColor(Qt::green);
        pB.setWidth(0.5);

        QPainter pointpainter(ui_->location_label);
        if (pointpainter.isActive())
        {
            pointpainter.setPen(pB);
            double px = pointx + poseA.x * w / 8, py = pointy - poseA.y * h / 8;
            QPointF point(px, py);
            points.append(point); // 将坐标点添加到存储列表中
            for (int i = 0; i < points.size() - 1; i++)
            {
                // QPointF p1 = points[i];
                // QPointF p2 = points[i + 1];
                // painter.drawLine(p1, p2);  // 连接相邻的坐标点
                QPointF p = points[i];
                pointpainter.drawPoint(p); // 逐个绘制点
            }
        }


        QPainter pointpainterB(ui_->location_label);
        if (pointpainterB.isActive())
        {
            pointpainterB.setPen(p1);
            double px_B = pointx + poseB.x * w / 8, py_B = pointy - poseB.y * h / 8;
            QPointF point_(px_B, py_B);
            points_B.append(point_); // 将坐标点添加到存储列表中
            for (int i = 0; i < points_B.size() - 1; i++)
            {
                // QPointF p1 = points[i];
                // QPointF p2 = points[i + 1];
                // painter.drawLine(p1, p2);  // 连接相邻的坐标点
                QPointF p_B = points_B[i];
                pointpainterB.drawPoint(p_B); // 逐个绘制点
            }
        }

        QPainter painter(ui_->location_label);  
        if (!painter.isActive())          
            painter.begin(ui_->location_label);
        if (painter.isActive())
        {
            painter.setPen(penDegree);
            painter.setFont(QFont(("黑体"), 10));
            painter.setRenderHint(QPainter::Antialiasing, true); // 设置反锯齿模式，好看一点
            // 绘制坐标轴 坐标轴原点(371-20，361-20)
            //  painter.drawRect(0,0,w,h);//外围的矩形
            // 画上x轴刻度线
            for (int i = 0; i < 9; i++) // 分成8份
            {
                // 选取合适的坐标，绘制一段长度为4的直线，用于表示刻度
                painter.drawLine(pointx + (i + 1) * width / 8, pointy, pointx + (i + 1) * width / 8, pointy + 4);
                painter.drawText(pointx - 5 + (i)*width / 8, pointy + 15, QString::number(i));
                painter.drawLine(pointx, pointy - (i + 1) * height / 8, pointx - 4, pointy - (i + 1) * height / 8);
                painter.drawText(pointx - 15, pointy + 8 - (i + 1) * height / 8, QString::number(i + 1));
            }
            painter.drawLine(pointx, pointy, width + pointx, pointy);  // 坐标轴x宽度为width
            painter.drawLine(pointx, pointy - height, pointx, pointy); // 坐标轴y高度为height
            
        }



        QPainter painter2(ui_->location_label);
        if (painter2.isActive())
        {
            painter2.setPen(penDegree2);
            for (int i = 0; i < 40; i++) // 分成5份
            {
                // 选取合适的坐标，绘制一段长度为4的直线，用于表示刻度
                painter2.drawLine(pointx + (i + 1) * width / 40, pointy, pointx + (i + 1) * width / 40, pointy - height);
                painter2.drawLine(pointx, pointy - (i + 1) * height / 40, pointx + width, pointy - (i + 1) * height / 40);
            }
        }

        

        
        

        update();
    }

    void VR_interaLsys::paintEventB(){

        // 设置画笔颜色
        QPen penDegree;
        penDegree.setColor(Qt::black);
        penDegree.setWidth(2);

        // 设置画笔颜色
        QPen penDegree2;
        penDegree2.setColor(Qt::gray);
        penDegree2.setWidth(0.01);

        QPen p1;
        p1.setColor(Qt::red);
        p1.setWidth(0.5);

        // QPainter pointpainterB(ui_->location_label_B);
        // if (pointpainterB.isActive())
        // {
        //     pointpainterB.setPen(p1);
        //     double px_B = pointx + poseB.x * w / 8, py_B = pointy - poseB.y * h / 8;
        //     QPointF point_(px_B, py_B);
        //     points_B.append(point_); // 将坐标点添加到存储列表中
        //     for (int i = 0; i < points_B.size() - 1; i++)
        //     {
        //         // QPointF p1 = points[i];
        //         // QPointF p2 = points[i + 1];
        //         // painter.drawLine(p1, p2);  // 连接相邻的坐标点
        //         QPointF p_B = points_B[i];
        //         pointpainterB.drawPoint(p_B); // 逐个绘制点
        //     }
        // }

        QPainter painterB(ui_->location_label_B);
        if (!painterB.isActive())
            painterB.begin(ui_->location_label_B);
        if (painterB.isActive())
        {
            
            painterB.setPen(penDegree);
            painterB.setFont(QFont(("黑体"), 10));
            painterB.setRenderHint(QPainter::Antialiasing, true); // 设置反锯齿模式，好看一点
            for (int i = 0; i < 9; i++)                           // 分成8份
            {
                painterB.drawLine(pointx + (i + 1) * width / 8, pointy, pointx + (i + 1) * width / 8, pointy + 4);
                painterB.drawText(pointx - 5 + (i)*width / 8, pointy + 15, QString::number(i));
                painterB.drawLine(pointx, pointy - (i + 1) * height / 8, pointx - 4, pointy - (i + 1) * height / 8);
                painterB.drawText(pointx - 15, pointy + 8 - (i + 1) * height / 8, QString::number(i + 1));
            }
            painterB.drawLine(pointx, pointy, width + pointx, pointy);  // 坐标轴x宽度为width
            painterB.drawLine(pointx, pointy - height, pointx, pointy); // 坐标轴y高度为height
            
        }

        // 网格线
        QPainter painter2B(ui_->location_label_B);
        if (painter2B.isActive())
        {
            painter2B.setPen(penDegree2);
            for (int i = 0; i < 40; i++) // 分成5份
            {
                painter2B.drawLine(pointx + (i + 1) * width / 40, pointy, pointx + (i + 1) * width / 40, pointy - height);
                painter2B.drawLine(pointx, pointy - (i + 1) * height / 40, pointx + width, pointy - (i + 1) * height / 40);
            }
        }

        update();
    }


    // void VR_interaLsys::paintEvent()
    // {
        
    //     //printf("paintEvent")
        
    //     // 设置画笔颜色
    //     QPen penDegree;
    //     penDegree.setColor(Qt::black);
    //     penDegree.setWidth(2);

    //     // 设置画笔颜色
    //     QPen penDegree2;
    //     penDegree2.setColor(Qt::gray);
    //     penDegree2.setWidth(0.01);

    //     QPen p1;
    //     p1.setColor(Qt::red);
    //     p1.setWidth(0.5);

    //     QPainter painter(ui_->location_label);
    //     painter.begin(ui_->location_label);
    //     painter.setPen(penDegree);       
    //     painter.setFont(QFont(("黑体"),10)); 
    //     painter.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式，好看一点        
    //     //绘制坐标轴 坐标轴原点(371-20，361-20)
    //     // painter.drawRect(0,0,w,h);//外围的矩形
    //     //画上x轴刻度线
    //     for(int i=0;i<9;i++)//分成8份
    //     {
    //         //选取合适的坐标，绘制一段长度为4的直线，用于表示刻度
    //         painter.drawLine(pointx+(i+1)*width/8,pointy,pointx+(i+1)*width/8,pointy+4);
    //         painter.drawText(pointx-5+(i)*width/8,pointy+15,QString::number(i));
    //         painter.drawLine(pointx,pointy-(i+1)*height/8,pointx-4,pointy-(i+1)*height/8);
    //         painter.drawText(pointx-15,pointy+8-(i+1)*height/8,QString::number(i+1));
    //     }
    //     QPainter painter2(ui_->location_label);
    //     painter2.setPen(penDegree2);
    //     for(int i=0;i<40;i++)//分成5份
    //     {
    //         //选取合适的坐标，绘制一段长度为4的直线，用于表示刻度
    //         painter2.drawLine(pointx+(i+1)*width/40,pointy,pointx+(i+1)*width/40,pointy-height);
    //         painter2.drawLine(pointx,pointy-(i+1)*height/40,pointx+width,pointy-(i+1)*height/40);

    //     }
    //     painter.drawLine(pointx,pointy,width+pointx,pointy);//坐标轴x宽度为width
    //     painter.drawLine(pointx,pointy-height,pointx,pointy);//坐标轴y高度为height
    //     QPainter pointpainter(ui_->location_label);
    //     pointpainter.setPen(p1);
    //     double px=pointx+poseA.x*w/8,py=pointy-poseA.y*h/8;
    //     QPointF point(px, py);
    //     points.append(point);  // 将坐标点添加到存储列表中
    //     for (int i = 0; i < points.size() - 1; i++) {
    //             // QPointF p1 = points[i];
    //             // QPointF p2 = points[i + 1];
    //             // painter.drawLine(p1, p2);  // 连接相邻的坐标点
    //             QPointF p = points[i];
    //             painter.drawPoint(p);  // 逐个绘制点
    //     }


    //     QPainter painterB(ui_->location_label_B);
    //     painterB.begin(ui_->location_label_B);
    //     painterB.setPen(penDegree);
    //     painterB.setFont(QFont(("黑体"),10)); 
    //     painterB.setRenderHint(QPainter::Antialiasing, true);//设置反锯齿模式，好看一点
    //     for(int i=0;i<9;i++)//分成8份
    //     {
    //         painterB.drawLine(pointx+(i+1)*width/8,pointy,pointx+(i+1)*width/8,pointy+4);
    //         painterB.drawText(pointx-5+(i)*width/8,pointy+15,QString::number(i));
    //         painterB.drawLine(pointx,pointy-(i+1)*height/8,pointx-4,pointy-(i+1)*height/8);
    //         painterB.drawText(pointx-15,pointy+8-(i+1)*height/8,QString::number(i+1));
    //     }
      
    //     //网格线
    //     QPainter painter2B(ui_->location_label_B);
    //     painter2B.setPen(penDegree2);
    //     for(int i=0;i<40;i++)//分成5份
    //     {
    //         painter2B.drawLine(pointx+(i+1)*width/40,pointy,pointx+(i+1)*width/40,pointy-height);
    //         painter2B.drawLine(pointx,pointy-(i+1)*height/40,pointx+width,pointy-(i+1)*height/40);

    //     }
    //     painterB.drawLine(pointx,pointy,width+pointx,pointy);//坐标轴x宽度为width
    //     painterB.drawLine(pointx,pointy-height,pointx,pointy);//坐标轴y高度为height
    //     QPainter pointpainterB(ui_->location_label_B);
    //     pointpainterB.setPen(p1);
    //     double px_B=pointx+poseB.x*w/8,py_B=pointy-poseB.y*h/8;
    //     QPointF point_(px_B, py_B);
    //     points_B.append(point_);  // 将坐标点添加到存储列表中
    //     for (int i = 0; i < points_B.size() - 1; i++) {
    //             // QPointF p1 = points[i];
    //             // QPointF p2 = points[i + 1];
    //             // painter.drawLine(p1, p2);  // 连接相邻的坐标点
    //             QPointF p_B = points_B[i];
    //             painterB.drawPoint(p_B);  // 逐个绘制点
    //     }
    //     update();

    // }
    
    
    
    bool VR_interaLsys::eventFilter(QObject *watched, QEvent *event) {
        if (watched == ui_->location_label&& event->type() == QEvent::Paint) {
            paintEvent();
        }
        if (watched == ui_->location_label_B&& event->type() == QEvent::Paint) {
           paintEventB();
        }
    
        return QWidget::eventFilter(watched, event);
    }

    
    void VR_interaLsys::A_front_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){

                    try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    QImage img = convertToQImage(imgCallback);
                    ui_->label_image->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image->width(),ui_->label_image->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }
            this->image_At_f.sec=msg->header.stamp.sec;
            this->image_At_f.nanosec=msg->header.stamp.nanosec;

            if(this->image_At_f.nanosec){
                struct timespec image_front_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_front_t);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/((image_front_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_front_t.tv_sec-this->lastTime.tv_sec));
                    
                    
                    this->ui_->imageFrontHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_front_t;
                int t0=image_front_t.tv_sec;
                int t1=image_front_t.tv_nsec;
                double image_front_delayt=(t0+t1*1e-9)-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
                this->ui_->imageFrontTime_label->setText(QString::number(image_front_delayt,'f',3));
                std::ofstream outfile("compress.txt",std::ios::app);  // 创建一个名为.txt 的输出文件流
                outfile<<"sec："<<msg->header.stamp.sec<<"    nanosec："<<msg->header.stamp.nanosec<<"    image_back_delayt:"<<image_front_delayt<<std::endl;

            }
    }
    void VR_interaLsys::B_front_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){
        try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    QImage img = convertToQImage(imgCallback);
                    ui_->label_image_front_B->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_front_B->width(),ui_->label_image_front_B->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }
            this->image_Bt_f.sec=msg->header.stamp.sec;
            this->image_Bt_f.nanosec=msg->header.stamp.nanosec;
            if(this->image_Bt_f.nanosec){
                struct timespec image_front_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_front_t);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/((image_front_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_front_t.tv_sec-this->lastTime.tv_sec));           
                    this->ui_->imageFrontHz_label_B->setText(QString::number(hz,'1',1));
                    
                }
                this->lastTime=image_front_t;
                int t0=image_front_t.tv_sec;
                int t1=image_front_t.tv_nsec;
                double image_front_delayt_B=(t0+t1*1e-9)-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
                this->ui_->imageFrontTime_label_B->setText(QString::number(image_front_delayt_B,'f',3));

                
                

            }
    }

    void VR_interaLsys::A_back_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){
        try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    QImage img = convertToQImage(imgCallback);
                    ui_->label_image_back->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back->width(),ui_->label_image_back->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }

            // std::cout<<"解析图片的时间为："<<msg2image_t<<std::endl;
            this->image_At_b.sec=msg->header.stamp.sec;
            this->image_At_b.nanosec=msg->header.stamp.nanosec;

            //delay

            //hz
            if(this->image_At_b.nanosec){
                struct timespec image_back_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_back_t);
                if(this->lastTime1.tv_sec!=0){
                    double hz=1/((image_back_t.tv_nsec-this->lastTime1.tv_nsec)*1e-9+(image_back_t.tv_sec-this->lastTime1.tv_sec));
                    this->ui_->imageBackHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime1=image_back_t;

                int t0=image_back_t.tv_sec;
                int t1=image_back_t.tv_nsec;
                double image_back_delayt=(t0+t1*1e-9)-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
                this->ui_->imageBackTime_label->setText(QString::number(image_back_delayt,'f',3));
                
            }
    }
    void VR_interaLsys::B_back_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg){
        try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    QImage img = convertToQImage(imgCallback);
                    ui_->label_image_back_B->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back_B->width(),ui_->label_image_back_B->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }

            // std::cout<<"解析图片的时间为："<<msg2image_t<<std::endl;
            this->image_Bt_b.sec=msg->header.stamp.sec;
            this->image_Bt_b.nanosec=msg->header.stamp.nanosec;

            //delay

            //hz
            if(this->image_Bt_b.nanosec){
                struct timespec image_back_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_back_t);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/((image_back_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_back_t.tv_sec-this->lastTime.tv_sec));

                    this->ui_->imageBackHz_label_B->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_back_t;

                int t0=image_back_t.tv_sec;
                int t1=image_back_t.tv_nsec;
                double image_back_delayt=(t0+t1*1e-9)-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
                this->ui_->imageBackTime_label_B->setText(QString::number(image_back_delayt,'f',3));
                
            }
    }
    
    void VR_interaLsys::V_back_image_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){

            // struct timespec t = {0, 0};
            // clock_gettime(CLOCK_REALTIME, &t);
            // int n1=t.tv_sec;
            // int n2=t.tv_nsec;
            // double image_delayt=(n1+n2 *1e-9)-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
            // std::cout<<rearimage_count<<": rawtime="<<image_delayt<<std::endl;
            // this->ui_->imageBackTime_label_v->setText(QString::number(image_delayt,'f',3));


            try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;                
                    QImage img = convertToQImage(imgCallback);
                    ui_->label_image_back_virtual->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back_virtual->width(),ui_->label_image_back_virtual->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }
            this->image_Bt_b.sec=msg->header.stamp.sec;
            this->image_Bt_b.nanosec=msg->header.stamp.nanosec;
            if(this->image_Bt_b.nanosec){
                struct timespec image_back_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_back_t);
                double time=(image_back_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_back_t.tv_sec-this->lastTime.tv_sec);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->imageBackHz_label_v->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_back_t;
                int n1=image_back_t.tv_sec;
                int n2=image_back_t.tv_nsec;
                double image_delayt=(n1+n2 *1e-9)-(msg->header.stamp.sec+msg->header.stamp.nanosec*1e-9);
                this->ui_->imageBackTime_label_v->setText(QString::number(image_delayt,'f',3));

            }

    
    }
    void VR_interaLsys::V_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){

            // 反序列化图像
            sensors_msg::ImageProto msg_pb = deserializeImage(ser_msg);
            // 将图像转换为 cv::Mat
            cv::Mat converted_image = convertToCVMat(msg_pb);
            // 显示图像在 UI 上
            QImage img(converted_image.data, converted_image.cols, converted_image.rows, 
                        converted_image.step, QImage::Format_RGB888);

            ui_->label_image_back_virtual->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image->width(), ui_->label_image->height()));
            this->image_At_f.sec=msg_pb.header_stamp_sec();
            this->image_At_f.nanosec=msg_pb.header_stamp_nanosec();            

            //delay

            //hz
            if(this->image_At_f.nanosec){
                struct timespec image_pb_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_pb_t);
                double time=(image_pb_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_pb_t.tv_sec-this->lastTime.tv_sec);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->imageBackHz_label_v->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_pb_t;

                 int n1=image_pb_t.tv_sec;
                int n2=image_pb_t.tv_nsec;
                double image_pb_delayt=(n1+n2 *1e-9)-(msg_pb.header_stamp_sec()+msg_pb.header_stamp_nanosec()*1e-9);

                this->ui_->imageBackTime_label_v->setText(QString::number(image_pb_delayt,'f',3));
            }

    }
    
    void VR_interaLsys::Af_ser_compressedimagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){
        std::cout<<"Compressedcamera_pb_Ar"<<std::endl;
        
        sensors_msg::ImageProto msg_pb = deserializeImage(ser_msg);
        struct timespec t = {0, 0};
        clock_gettime(CLOCK_REALTIME, &t);
        int t0=t.tv_sec,t1=t.tv_nsec;
        sensor_msgs::msg::CompressedImage imgc;
        imgc.set__format(msg_pb.format());
        imgc.header.frame_id=msg_pb.header_frame_id();
        imgc.header.stamp.nanosec=msg_pb.header_stamp_nanosec();
        imgc.header.stamp.sec=msg_pb.header_stamp_sec();
        imgc.data.resize(msg_pb.data().size());
        memcpy(imgc.data.data(),msg_pb.data().data(),msg_pb.data().size());

                try
                {
                    cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(imgc,sensor_msgs::image_encodings::BGR8);
                    cv::Mat imgCallback;
                    imgCallback = cv_ptr_compressed->image;
                    QImage img = convertToQImage(imgCallback);
                    ui_->label_image_back->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back->width(),ui_->label_image_back->height()));
                    //cv::waitKey(1);
                   
                }
            catch (cv_bridge::Exception& e)
                {
                //ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
                }

        this->image_Bt_f.sec=imgc.header.stamp.sec;
            this->image_Bt_f.nanosec=imgc.header.stamp.nanosec;
            if(this->image_Bt_f.nanosec){
                struct timespec image_front_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_front_t);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/((image_front_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_front_t.tv_sec-this->lastTime.tv_sec));
                    this->ui_->imageBackHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_front_t;
                int t0=image_front_t.tv_sec;
                int t1=image_front_t.tv_nsec;
                double image_front_delayt_B=(t0+t1*1e-9)-(imgc.header.stamp.sec+imgc.header.stamp.nanosec*1e-9);
                this->ui_->imageBackTime_label->setText(QString::number(image_front_delayt_B,'f',3));
                std::ofstream outfile("compress_ser.txt",std::ios::app);  // 创建一个名为.txt 的输出文件流
                outfile<<"sec："<<imgc.header.stamp.sec<<"    nanosec："<<imgc.header.stamp.nanosec<<"    image_back_delayt:"<<image_front_delayt_B<<std::endl;
            }

            


    }
    
    void VR_interaLsys::Af_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){

            // 反序列化图像
            sensors_msg::ImageProto msg_pb = deserializeImage(ser_msg);
            // 将图像转换为 cv::Mat
            cv::Mat converted_image = convertToCVMat(msg_pb);
            // 显示图像在 UI 上
            QImage img(converted_image.data, converted_image.cols, converted_image.rows, 
                        converted_image.step, QImage::Format_RGB888);

            ui_->label_image->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image->width(), ui_->label_image->height()));
            this->image_At_f.sec=msg_pb.header_stamp_sec();
            this->image_At_f.nanosec=msg_pb.header_stamp_nanosec();            

            //delay

            //hz
            if(this->image_At_f.nanosec){
                struct timespec image_pb_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_pb_t);
                double time=(image_pb_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_pb_t.tv_sec-this->lastTime.tv_sec);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->imageFrontHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_pb_t;

                 int n1=image_pb_t.tv_sec;
                int n2=image_pb_t.tv_nsec;
                double image_pb_delayt=(n1+n2 *1e-9)-(msg_pb.header_stamp_sec()+msg_pb.header_stamp_nanosec()*1e-9);

                this->ui_->imageFrontTime_label->setText(QString::number(image_pb_delayt,'f',3));
            }

    }
    void VR_interaLsys::Bf_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){

            // 反序列化图像
            sensors_msg::ImageProto msg_pb = deserializeImage(ser_msg);
            // 将图像转换为 cv::Mat
            cv::Mat converted_image = convertToCVMat(msg_pb);
            // 显示图像在 UI 上
            QImage img(converted_image.data, converted_image.cols, converted_image.rows, 
                        converted_image.step, QImage::Format_RGB888);

            ui_->label_image_front_B->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_front_B->width(), ui_->label_image_front_B->height()));
            this->image_At_f.sec=msg_pb.header_stamp_sec();
            this->image_At_f.nanosec=msg_pb.header_stamp_nanosec();            

            //delay

            //hz
            if(this->image_At_f.nanosec){
                struct timespec image_pb_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_pb_t);
                double time=(image_pb_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_pb_t.tv_sec-this->lastTime.tv_sec);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->imageFrontHz_label_B->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_pb_t;

                 int n1=image_pb_t.tv_sec;
                int n2=image_pb_t.tv_nsec;
                double image_pb_delayt=(n1+n2 *1e-9)-(msg_pb.header_stamp_sec()+msg_pb.header_stamp_nanosec()*1e-9);

                this->ui_->imageFrontTime_label_B->setText(QString::number(image_pb_delayt,'f',3));
            }

    }
    void VR_interaLsys::Ar_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){

            // 反序列化图像
            sensors_msg::ImageProto msg_pb = deserializeImage(ser_msg);
            // 将图像转换为 cv::Mat
            cv::Mat converted_image = convertToCVMat(msg_pb);
            // 显示图像在 UI 上
            QImage img(converted_image.data, converted_image.cols, converted_image.rows, 
                        converted_image.step, QImage::Format_RGB888);

            ui_->label_image_back->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back->width(), ui_->label_image_back->height()));
            this->image_At_f.sec=msg_pb.header_stamp_sec();
            this->image_At_f.nanosec=msg_pb.header_stamp_nanosec();            

            //delay

            //hz
            if(this->image_At_f.nanosec){
                struct timespec image_pb_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_pb_t);
                double time=(image_pb_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_pb_t.tv_sec-this->lastTime.tv_sec);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->imageBackHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_pb_t;

                 int n1=image_pb_t.tv_sec;
                int n2=image_pb_t.tv_nsec;
                double image_pb_delayt=(n1+n2 *1e-9)-(msg_pb.header_stamp_sec()+msg_pb.header_stamp_nanosec()*1e-9);

                this->ui_->imageBackTime_label->setText(QString::number(image_pb_delayt,'f',3));
            }

    }
    void VR_interaLsys::Br_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){

            // 反序列化图像
            sensors_msg::ImageProto msg_pb = deserializeImage(ser_msg);
            // 将图像转换为 cv::Mat
            cv::Mat converted_image = convertToCVMat(msg_pb);
            // 显示图像在 UI 上
            QImage img(converted_image.data, converted_image.cols, converted_image.rows, 
                        converted_image.step, QImage::Format_RGB888);

            ui_->label_image_back_B->setPixmap(QPixmap::fromImage(img).scaled(ui_->label_image_back_B->width(), ui_->label_image_back_B->height()));
            this->image_At_f.sec=msg_pb.header_stamp_sec();
            this->image_At_f.nanosec=msg_pb.header_stamp_nanosec();            

            //delay

            //hz
            if(this->image_At_f.nanosec){
                struct timespec image_pb_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &image_pb_t);
                double time=(image_pb_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(image_pb_t.tv_sec-this->lastTime.tv_sec);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->imageBackHz_label_B->setText(QString::number(hz,'1',1));
                }
                this->lastTime=image_pb_t;

                int n1=image_pb_t.tv_sec;
                int n2=image_pb_t.tv_nsec;
                double image_pb_delayt=(n1+n2 *1e-9)-(msg_pb.header_stamp_sec()+msg_pb.header_stamp_nanosec()*1e-9);

                this->ui_->imageBackTime_label_B->setText(QString::number(image_pb_delayt,'f',3));
            }

    }
    
    void VR_interaLsys::imudata_callback(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line){
        // 创建tf2四元数对象
        tf2::Quaternion quaternion;
        quaternion.setX(actualData_line->orientation.x);
        quaternion.setY(actualData_line->orientation.y);
        quaternion.setZ(actualData_line->orientation.z);
        quaternion.setW(actualData_line->orientation.w);
        // 将四元数转换为tf2矩阵
        tf2::Matrix3x3 matrix(quaternion);

        // 计算欧拉角（俯仰、侧倾和航向）
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        ui_->roll->setText(QString::number(roll,'f',2));
        ui_->pitch->setText(QString::number(pitch,'f',2));
        ui_->yaw->setText(QString::number(yaw,'f',2));


        imu_A.Acc.x=actualData_line->linear_acceleration.x;
        imu_A.Acc.y=actualData_line->linear_acceleration.y;
        imu_A.Acc.z=actualData_line->linear_acceleration.z;

        imu_A.Angual.x=actualData_line->angular_velocity.x;
        imu_A.Angual.y=actualData_line->angular_velocity.y;
        imu_A.Angual.z=actualData_line->angular_velocity.z;

        this->imu_At.sec=actualData_line->header.stamp.sec;
        this->imu_At.nanosec=actualData_line->header.stamp.nanosec;

        if (this->imu_At.nanosec)
        {
            struct timespec timeImu = {0, 0};
            clock_gettime(CLOCK_REALTIME, &timeImu);
            if (this->lastTime2.tv_sec!=0)
            {
                //如果不等于0，有数据传入
                //当前时间减去上一刻时间
                double hz=1/((timeImu.tv_sec - this->lastTime2.tv_sec)+(timeImu.tv_nsec-this->lastTime2.tv_nsec)*1e-9);
                this->ui_->imuHz_label->setText(QString::number(hz,'1',1));
            }
            
            this->lastTime2=timeImu;

            int t0=timeImu.tv_sec;
            int t1=timeImu.tv_nsec;
            float imu_delayt=(t0+t1*1e-9)-(actualData_line->header.stamp.sec+actualData_line->header.stamp.nanosec*1e-9);
            this->ui_->imuTime_label->setText(QString::number(imu_delayt,'f',3));

        }

    }
    void VR_interaLsys::imumag_callback(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line){
        imu_A.Mag.x=actualMagData_line->magnetic_field.x;
        imu_A.Mag.y=actualMagData_line->magnetic_field.y;
        imu_A.Mag.z=actualMagData_line->magnetic_field.z;
    }
    
    void VR_interaLsys::imudata_callback_B(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line){
        // 创建tf2四元数对象
        tf2::Quaternion quaternion;
        quaternion.setX(actualData_line->orientation.x);
        quaternion.setY(actualData_line->orientation.y);
        quaternion.setZ(actualData_line->orientation.z);
        quaternion.setW(actualData_line->orientation.w);
        // 将四元数转换为tf2矩阵
        tf2::Matrix3x3 matrix(quaternion);

        // 计算欧拉角（俯仰、侧倾和航向）
        double roll, pitch, yaw;
        matrix.getRPY(roll, pitch, yaw);
        ui_->roll_B->setText(QString::number(roll,'f',2));
        ui_->pitch_B->setText(QString::number(pitch,'f',2));
        ui_->yaw_B->setText(QString::number(yaw,'f',2));


        imu_B.Acc.x=actualData_line->linear_acceleration.x;
        imu_B.Acc.y=actualData_line->linear_acceleration.y;
        imu_B.Acc.z=actualData_line->linear_acceleration.z;

        imu_B.Angual.x=actualData_line->angular_velocity.x;
        imu_B.Angual.y=actualData_line->angular_velocity.y;
        imu_B.Angual.z=actualData_line->angular_velocity.z;

        this->imu_At.sec=actualData_line->header.stamp.sec;
        this->imu_At.nanosec=actualData_line->header.stamp.nanosec;

        if (this->imu_At.nanosec)
        {
            struct timespec timeImu = {0, 0};
            clock_gettime(CLOCK_REALTIME, &timeImu);
            if (this->lastTime.tv_sec!=0)
            {
                //如果不等于0，有数据传入
                //当前时间减去上一刻时间
                double hz=1/((timeImu.tv_sec - this->lastTime.tv_sec)+(timeImu.tv_nsec-this->lastTime.tv_nsec)*1e-9);             
                this->ui_->imuHz_label_B->setText(QString::number(hz,'1',1));
                std::ofstream outfile("imu.txt",std::ios::app);  // 创建一个名为.txt 的输出文件流
                outfile<<"sec："<<actualData_line->header.stamp.sec<<"    nanosec："<<actualData_line->header.stamp.nanosec<<"    hz:"<<hz
                <<"   lastTime: " << this->lastTime.tv_nsec <<" "<< this->lastTime.tv_sec << "    now time "<< timeImu.tv_nsec<<" "<<timeImu.tv_sec<<std::endl;
            }
            
            this->lastTime=timeImu;

            int t0=timeImu.tv_sec;
            int t1=timeImu.tv_nsec;
            float imu_delayt=(t0+t1*1e-9)-(actualData_line->header.stamp.sec+actualData_line->header.stamp.nanosec*1e-9);
            this->ui_->imuTime_label_B->setText(QString::number(imu_delayt,'f',3));
             

        }

    }
    void VR_interaLsys::imumag_callback_B(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line){
        imu_B.Mag.x=actualMagData_line->magnetic_field.x;
        imu_B.Mag.y=actualMagData_line->magnetic_field.y;
        imu_B.Mag.z=actualMagData_line->magnetic_field.z;
    }
    
    void VR_interaLsys::pointcloud2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg){
        

        this->cloud_At.sec=pointcloud2_msg->header.stamp.sec;
        this->cloud_At.nanosec=pointcloud2_msg->header.stamp.nanosec;
        
        if(this->cloud_At.nanosec){
                struct timespec pC2_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &pC2_t);
                double time=pointcloud2_msg->header.stamp.sec+pointcloud2_msg->header.stamp.nanosec*1e-9;
                if(this->lastTime.tv_sec!=0){
                    double hz=1/(pC2_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(pC2_t.tv_sec-this->lastTime.tv_sec);
                    this->ui_->cloudHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime=pC2_t;

                int t0=pC2_t.tv_sec;
                int t1=pC2_t.tv_nsec;
                double pC2_delayt=(t0+t1*1e-9)-(pointcloud2_msg->header.stamp.sec+pointcloud2_msg->header.stamp.nanosec*1e-9);
                this->ui_->cloudTime_label->setText(QString::number(pC2_delayt,'f',3));

        }
        

        
    }
    void VR_interaLsys::pointcloud2pb_callback_B(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){

        pointcloud2::pointCloud2_pb msg_pb;
        std::vector<uint8_t> msg_data(ser_msg->data.begin(), ser_msg->data.end());

        const auto pb_start_t=std::chrono::high_resolution_clock::now();
        msg_pb.ParseFromArray(msg_data.data(),msg_data.size());
        const auto pb_end_t=std::chrono::high_resolution_clock::now();
        const auto dser_t=std::chrono::duration_cast<std::chrono::nanoseconds>(pb_end_t-pb_start_t).count();

        std::cout<<"pb_size:"<<msg_pb.data().size()<<"  高:"<<msg_pb.height()<<"  宽:"<<msg_pb.width()<<"   每个点的字节数:"<<msg_pb.point_step()<<
        "  消息中每行的字节数:"<<msg_pb.row_step()<<"   frame_id:"<<msg_pb.frame_id().c_str()<<"   点云数据是否密集:"<<msg_pb.is_dense()<<"   点云数据的字节序:"<<msg_pb.is_bigendian()<<
        "   字段的名称:"<<msg_pb.fields().size()<<std::endl;
        
        sensor_msgs::msg::PointCloud2 cloud;
        using PointField = sensor_msgs::msg::PointField;   
        cloud.height = 1;
        cloud.width = 0;
        cloud.fields.resize(6);
        cloud.fields[0].offset = 0;
        cloud.fields[0].name = "x";
        cloud.fields[0].count = 1;
        cloud.fields[0].datatype = PointField::FLOAT32;
        cloud.fields[1].offset = 4;
        cloud.fields[1].name = "y";
        cloud.fields[1].count = 1;
        cloud.fields[1].datatype = PointField::FLOAT32;
        cloud.fields[2].offset = 8;
        cloud.fields[2].name = "z";
        cloud.fields[2].count = 1;
        cloud.fields[2].datatype = PointField::FLOAT32;
        cloud.fields[3].offset = 12;
        cloud.fields[3].name = "intensity";
        cloud.fields[3].count = 1;
        cloud.fields[3].datatype = PointField::FLOAT32;
        cloud.fields[4].offset = 16;
        cloud.fields[4].name = "tag";
        cloud.fields[4].count = 1;
        cloud.fields[4].datatype = PointField::UINT8;
        cloud.fields[5].offset = 17;
        cloud.fields[5].name = "line";
        cloud.fields[5].count = 1;
        cloud.fields[5].datatype = PointField::UINT8;
        cloud.point_step = msg_pb.point_step();
        cloud.width=msg_pb.width();
        cloud.row_step=msg_pb.row_step();
        cloud.header.frame_id=msg_pb.frame_id();
        cloud.header.stamp.nanosec=msg_pb.stamp_nanosec();
        cloud.header.stamp.sec=msg_pb.stamp_sec();
        cloud.is_bigendian=msg_pb.is_bigendian();
        cloud.is_dense=msg_pb.is_dense();
        

        // std::vector<LivoxPointXyzrtl> points;//注释
        // std::vector<pointcloud2::point_pb> pointpbs;
        // for (size_t i = 0; i < cloud.width; ++i) {
        //     LivoxPointXyzrtl point;
        //     point.x=pointpbs[i].x();
        //     point.y=pointpbs[i].y();
        //     point.z=pointpbs[i].z();
        //     point.reflectivity=pointpbs[i].reflectivity();
        //     point.tag=pointpbs[i].tag();
        //     point.line=pointpbs[i].line();

        //     points.push_back(std::move(point));
        // }

        
        cloud.data.resize(msg_pb.data().size());
        memcpy(cloud.data.data(),msg_pb.data().data(),msg_pb.data().size());
        
        pb_2_cloud2_B->publish(cloud);

      
        this->cloud_At.nanosec=msg_pb.stamp_nanosec();
        this->cloud_At.sec=msg_pb.stamp_sec();

        if(this->cloud_At.nanosec){
            rclcpp::Time now = node_->get_clock()->now();
            builtin_interfaces::msg::Time tt = now;
                struct timespec pC2_pb_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &pC2_pb_t);
                double time=(pC2_pb_t.tv_nsec-this->lastTime.tv_nsec)*1e-9+(pC2_pb_t.tv_sec-this->lastTime.tv_sec);
                if(this->lastTime.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->cloudHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime=pC2_pb_t;

                int t0=pC2_pb_t.tv_sec;
                int t1=pC2_pb_t.tv_nsec;
                double pC2_pb_delayt=(tt.sec+tt.nanosec*1e-9)-(msg_pb.stamp_sec()+msg_pb.stamp_nanosec()*1e-9);
                this->ui_->cloudTime_label->setText(QString::number(pC2_pb_delayt,'f',3));
        }

        

    }

    
    void VR_interaLsys::pointcloud2pb_callback_A(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg){

        pointcloud2::pointCloud2_pb msg_pb = deserializePointCloud2(ser_msg);
        
      
        this->cloud_At.nanosec=msg_pb.stamp_nanosec();
        this->cloud_At.sec=msg_pb.stamp_sec();

        if(this->cloud_At.nanosec){
            rclcpp::Time now = node_->get_clock()->now();
            builtin_interfaces::msg::Time tt = now;
                struct timespec pC2_pb_t = {0, 0};
                clock_gettime(CLOCK_REALTIME, &pC2_pb_t);
                double time=(pC2_pb_t.tv_nsec-this->lastTime3.tv_nsec)*1e-9+(pC2_pb_t.tv_sec-this->lastTime3.tv_sec);
                if(this->lastTime3.tv_sec!=0){
                    double hz=1/time;
                    this->ui_->cloudHz_label->setText(QString::number(hz,'1',1));
                }
                this->lastTime3=pC2_pb_t;

                int t0=pC2_pb_t.tv_sec;
                int t1=pC2_pb_t.tv_nsec;
                double pC2_pb_delayt=(tt.sec+tt.nanosec*1e-9)-(msg_pb.stamp_sec()+msg_pb.stamp_nanosec()*1e-9);
                this->ui_->cloudTime_label->setText(QString::number(pC2_pb_delayt,'f',3));
        }

        sensor_msgs::msg::PointCloud2 cloud = convertPointCloud2(msg_pb);
        pb_2_cloud2_A->publish(cloud);
    }

    // void VR_interaLsys::mqtt_receive(){
    //     signal(SIGINT, handle_signal);
    //     signal(SIGTERM, handle_signal);

    //     mosquitto_lib_init();

    //     struct mosquitto *mosq = mosquitto_new("subscriber", true, NULL);

    //     if (mosq)
    //     {
    //         mosquitto_connect_callback_set(mosq, connect_callback);
    //         mosquitto_message_callback_set(mosq, message_callback);

    //         int rc = mosquitto_connect(mosq, "127.0.0.1", 1883, 60);

    //         mosquitto_subscribe(mosq, NULL, "sensor/temperature", 0);
    //         mosquitto_subscribe(mosq, NULL, "sensor/people_count", 0);
    //         mosquitto_subscribe(mosq, NULL, "sensor/fan_speed", 0);

    //         while (alive)
    //         {
    //         if (alive && rc) {
    //             mosquitto_reconnect(mosq);
    //         }

    //         rc = mosquitto_loop(mosq, -1, 1);
    //         }

    //         mosquitto_destroy(mosq);
    //     }

    //     mosquitto_lib_cleanup();
    // }
    // void VR_interaLsys::message_callback(struct mosquitto *mosq, void *obj,const struct mosquitto_message *message)
    // {
    //         if (!strcmp(message->topic, "sensor/temperature")) {
    //             std::cout << "Temperature: " << (char *)message->payload << std::endl;
    //         }

    //         if (!strcmp(message->topic, "sensor/people_count")) {
    //             std::cout << "People Count: " << (char *)message->payload << std::endl;
    //         }

    //         if (!strcmp(message->topic, "sensor/fan_speed")) {
    //             std::cout << "Fan Speed: " << (char *)message->payload << std::endl;
    //         }
    // }

    // void VR_interaLsys::mqtt_send(){
    //     // 注册信号处理程序
    //     signal(SIGINT, handle_signal);
    //     signal(SIGTERM, handle_signal);

    //     // 初始化 Mosquitto 库
    //     mosquitto_lib_init();
    //     std::cout<<"mosquitto_lib_init"<<std::endl;
        
    //     struct mosquitto *mosq;

    //     // 创建 mosquitto 实例  第一个参数是 client_id，表示需要分配给该 mosquitto 实例的客户端ID。
    //     mosq = mosquitto_new("people_count_publisher", true, NULL);

    //     if (mosq)
    //     {
    //         mosquitto_connect_callback_set(mosq, connect_callback);

    //         // 连接到 MQTT 服务器。第二个参数是代理的主机名或IP地址，本例中将其设置为"localhost"，表示连接到本地主机。
    //         //第三个参数是代理的端口号，本例中设置为1883，这是MQTT协议的默认端口。第四个参数是keepalive值，表示与代理之间的心跳间隔时间
    //         //成功连接时，rc将为0。连接失败或出现错误时，rc将为非零值，可以根据具体错误代码进行调试和处理。
    //         int rc = mosquitto_connect(mosq, "192.168.1.110", 1883, 60);
    //         std::cout<<"rc"<<rc<<std::endl;


    //         while (alive)
    //         {
    //         if (alive && rc) {
    //             connected = false;
    //             mosquitto_reconnect(mosq);
    //         }

    //         if (connected)
    //         {
    //             std::stringstream ss;

    //             int random = (rand() % 3) + 1;
    //             ss << random;

    //             auto message = ss.str();
    //         std::cout<<"ss"<<ss.str()<<std::endl;
                

    //             rc = mosquitto_publish(
    //             mosq, NULL, "sensor/people_count", message.size(), message.c_str(), 0, true
    //             );

    //             if (!rc) {
    //             std::cout << "Published: " << message << std::endl;
    //             sleep(5);
    //             }
    //         }

    //         // 处理网络事件 如果返回MOSQ_ERR_SUCCESS，表示成功处理了一个事件。其他返回值表示发生了错误或连接断开等情况。
    //         rc = mosquitto_loop(mosq, -1, 1);
    //     }

    //         // 清理 mosquitto 实例
    //         mosquitto_destroy(mosq);
    //     }

    //     // 释放 Mosquitto 库资源
    //     mosquitto_lib_cleanup();
    // }
    // void VR_interaLsys::handle_signal(int s)
    // {
    //     VR_interaLsys::alive=false;
    //     // alive = false;
    // }

    // void VR_interaLsys::connect_callback(struct mosquitto *mosq_, void *obj, int result)
    // {
    //     std::cout << "Connected to MQTT broker" << std::endl;
    //     VR_interaLsys::connected=true;
    //     // connected=true;
    // }

    void VR_interaLsys::UDP_send(){
        /* socket文件描述符 */
        int sock_fd;

        /* 建立udp socket */
        sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sock_fd < 0)
        {
            perror("socket");
            exit(1);
        }

        /* 设置address */
        struct sockaddr_in addr_serv;
        int len;
        memset(&addr_serv, 0, sizeof(addr_serv));
        addr_serv.sin_family = AF_INET;
        addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);
        addr_serv.sin_port = htons(DEST_PORT);

        while(1)
        {
            char buff[128]={0};
            printf("input:\n");
            fgets(buff,128,stdin);
            if(strncmp(buff,"end",3) ==0 )
            {
                break;
            }
            sendto(sock_fd, buff, strlen(buff), 0, (struct sockaddr *)&addr_serv, sizeof(addr_serv));
            memset(buff,0,128);
            int len = sizeof(addr_serv);

            recvfrom(sock_fd,buff,127,0,(struct sockaddr*)&addr_serv,(socklen_t*)&len);
            printf("buff=%\n",buff);

        }
    }
    
    void VR_interaLsys::UDP_receive(){
        //socket接收消息
        
        /* sock_fd --- socket文件描述符 创建udp套接字*/
        int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if(sock_fd < 0)
        {
            perror("socket");
            exit(1);
        }

        /* 将套接字和IP、端口绑定 */
        struct sockaddr_in addr_serv;
        memset(&addr_serv, 0, sizeof(struct sockaddr_in));  //每个字节都用0填充
        addr_serv.sin_family = AF_INET;                     //使用IPV4地址
        addr_serv.sin_port = htons(8000);              //本地端口 普通数字可以用htons()函数转换成网络数据格式的数字
                                                            /* INADDR_ANY表示不管是哪个网卡接收到数据，只要目的端口是SERV_PORT，就会被该应用程序接收到 */
        addr_serv.sin_addr.s_addr = inet_addr(DSET_IP_ADDRESS);      //自动获取IP地址 将点分十进制的IP地址转化为二进制

        /* 绑定socket */
        if(bind(sock_fd, (struct sockaddr *)&addr_serv, sizeof(addr_serv)) < 0)
        {
            perror("bind error:");
            exit(1);
        }

        struct sockaddr_in addr_client;
        std::vector<char> received_data;

        int total_packets=0;
        while (1) {
                // 接收数据包数量信息
            
            std::vector<char> total_packets_data(sizeof(total_packets));
            socklen_t addrLen = sizeof(addr_client);
            ssize_t bytes_count_received = recvfrom(sock_fd, total_packets_data.data(), total_packets_data.size(), 0, (struct sockaddr*)&addr_client, &addrLen);
            memcpy(&total_packets, total_packets_data.data(), sizeof(total_packets));

            
            // int received_packet_count=0;
            for (int received_packet_count=0;received_packet_count<total_packets;received_packet_count++)
            {
                 // 存储接收到的所有数据包的内容
                received_data.resize(total_packets * MAX_PACKET_SIZE);

                
                std::vector<char> packet_data(MAX_PACKET_SIZE);
                ssize_t bytes_received = recvfrom(sock_fd, packet_data.data(), packet_data.size(), 0, (struct sockaddr*)&addr_client, &addrLen);
                std::cout << "packages received" << std::endl;

                memcpy(&received_data[(received_packet_count) * MAX_PACKET_SIZE], packet_data.data(), bytes_received);
                std::cout << "total_packets:"<<total_packets<<"  received_packet_count: " << received_packet_count << "   packet_data size: " << bytes_received << std::endl;

                // received_packet_count++;
                if (received_packet_count == total_packets-1) {
                    std::cout << "使用 Protocol Buffers 进行反序列化操作" << std::endl;
                    // 重置状态以接收下一批数据
                    received_packet_count=0;
                    received_data.clear();
                    total_packets=0;
                    total_packets_data.clear();
                    break;
                }

                
            }
            
            
            
            
            
        }
        received_data.clear();
        total_packets = 0;

        // for (int received_packet_count=0;received_packet_count<total_packets;received_packet_count++){
        //     std::vector<char> packet_data(MAX_PACKET_SIZE);
        //     ssize_t bytes_received = recvfrom(sock_fd, packet_data.data(), packet_data.size(), 0, (struct sockaddr*)&addr_client, &addrLen);
        //     std::cout<<"packages received"<<std::endl;

        //     memcpy(&received_data[(received_packet_count) * MAX_PACKET_SIZE], packet_data.data(), bytes_received);
        //     std::cout<<"received_packet_count:"<<received_packet_count<<"   packet_data size:"<<bytes_received<<std::endl;
        //     if (received_packet_count == total_packets-1) {
        //         std::cout<<"使用 Protocol Buffers 进行反序列化操作"<<std::endl;
        //         // 重置状态以接收下一批数据
        //         received_packet_count = 0;
        //         total_packets = 0;
        //         received_data.clear();
        //         break;
                
        //     }

        // }
        


        // pointcloud2::pointCloud2_pb msg_pb;
        // msg_pb.ParseFromArray(received_data.data(),received_data.size());
        // // printf("ip=%s,port=%d,buff=%s\n",inet_ntoa(addr_client.sin_addr),ntohs(addr_client.sin_port),msg_pb.width());
        // std::cout<<"  msg_pb size:"<<msg_pb.data().size()<<"  width:"<<msg_pb.width()<<"   height:"<<msg_pb.height()<<std::endl;
        // // sendto(sock_fd,"OK",2,0,(struct sockaddr *)&addr_client, sizeof(addr_client));
        // received_data.clear();  // 清空容器内容

    }
    
    void VR_interaLsys::onTabCloseRequested(int index){
        A_tabWidget->removeTab(index);
    }
    
    void VR_interaLsys::set_tabWidget_test(QTabWidget *widget,imuchart& imu_chart,imuchart& imu_chart2,imuchart& imu_chart3,QLabel *chartlabel){

        widget->setParent(chartlabel);
        // A_tabWidget.setWindowTitle("yemian1");
        widget->resize(413,174);
        widget->setTabPosition(QTabWidget::North);
        widget->setTabShape(QTabWidget::Triangular);
        widget->setTabsClosable(false);

        // set font;
        labelsFont.setPixelSize(12);   //参数字号，数字越小，字就越小
        
        //Acc_line
        // 创建横纵坐标轴并设置显示范围

        set_axis(imu_chart.ho,imu_chart.lo);
        imu_chart.lo->setTitleText("Accelerate");
        imu_chart.lo->setRange(-10, 10);
        imu_chart.lo->setMax(AXIS_MAX_Y);
        imu_chart.ho->setMax(AXIS_MAX_X);
        set_series(imu_chart.x,imu_chart.y,imu_chart.z);
        set_chart(imu_chart.chart,imu_chart.ho,imu_chart.lo,imu_chart.x,imu_chart.y,imu_chart.z);
        set_attach(imu_chart.x,imu_chart.y,imu_chart.z,imu_chart.ho,imu_chart.lo);
               
        //Angual_line
        set_axis(imu_chart2.ho,imu_chart2.lo);
        imu_chart2.lo->setTitleText("Angual");
        // imu_chart2.lo->setMin(0);
        imu_chart2.lo->setRange(-10, 10);
        imu_chart2.ho->setMax(0);
        imu_chart2.lo->setMax(AXIS_MAX_Y);
        imu_chart2.ho->setMax(AXIS_MAX_X);
        set_series(imu_chart2.x,imu_chart2.y,imu_chart2.z);
        set_chart(imu_chart2.chart,imu_chart2.ho,imu_chart2.lo,imu_chart2.x,imu_chart2.y,imu_chart2.z);
        set_attach(imu_chart2.x,imu_chart2.y,imu_chart2.z,imu_chart2.ho,imu_chart2.lo);                           
        //printf("\n1488\n");
        //Mag_line
        set_axis(imu_chart3.ho,imu_chart3.lo);
        imu_chart3.lo->setTitleText("Mag");
        imu_chart3.lo->setRange(-2, 2);
        imu_chart3.lo->setMax(MAG_AXIS_MAX_Y);
        imu_chart3.ho->setMax(MAG_AXIS_MAX_X);
        set_series(imu_chart3.x,imu_chart3.y,imu_chart3.z);
        set_chart(imu_chart3.chart,imu_chart3.ho,imu_chart3.lo,imu_chart3.x,imu_chart3.y,imu_chart3.z);
        set_attach(imu_chart3.x,imu_chart3.y,imu_chart3.z,imu_chart3.ho,imu_chart3.lo); 
        //printf("\n1498\n");
        QtCharts::QChartView * yemian1 = new QtCharts::QChartView (widget);
        yemian1->setChart(imu_chart.chart); 
        widget->addTab(yemian1,"Acc");
        yemian1->setRenderHint(QPainter::Antialiasing);                 // 设置渲染：抗锯齿，如果不设置那么曲线就显得不平滑
        //printf("\n1503\n");
        QtCharts::QChartView * yemian2 = new QtCharts::QChartView (widget);
        yemian2->setChart(imu_chart2.chart); 
        widget->addTab(yemian2,"Angual");
        widget->setCurrentIndex(1);
        // ui_->Angual_graphicsView->setChart(Angual_chart);    
        yemian2->setRenderHint(QPainter::Antialiasing);

        QtCharts::QChartView * yemian3 = new QtCharts::QChartView (widget);
        yemian3->setChart(imu_chart3.chart); 
        widget->addTab(yemian3,"Mag");
        widget->setCurrentIndex(2);
        // ui_->Mag_graphicsView->setChart(Mag_chart);
        yemian3->setRenderHint(QPainter::Antialiasing);
        //printf("\n1517\n");
    }
    
    void VR_interaLsys::set_axis(QValueAxis *axisx,QValueAxis *axisy){
        axisx->setLabelsFont(labelsFont);
        axisy->setLabelsFont(labelsFont);
        axisx->setTitleText("time");
        // axisy->setTitleText("Accelerate");
        // axisy->setMin(0);
        axisy->setRange(-10, 10);
        axisx->setMax(0);
        // axisy->setMax(AXIS_MAX_Y);
        // axisx->setMax(AXIS_MAX_X);
        axisx->setMinorTickCount(4);
        axisx->setGridLineVisible(true);

    }
    void VR_interaLsys::set_series(QLineSeries* seriesx,QLineSeries* seriesy,QLineSeries* seriesz){

        seriesx->setPointsVisible(false);                 // 设置数据点可见
        seriesx->setName("X");                            // 图例名称
        seriesy->setPointsVisible(false);
        seriesy->setName("Y");
        seriesz->setPointsVisible(false);
        seriesz->setName("Z");

    }
    void VR_interaLsys::set_chart(QChart* chart,QValueAxis *axisx,QValueAxis *axisy,QLineSeries* seriesx,QLineSeries* seriesy,QLineSeries* seriesz){

        chart->legend()->setVisible(false);
        chart->setBackgroundRoundness(0);
        chart->setContentsMargins(0, 0, 0, 0);
        chart->setMargins(QMargins(0,0,0,0));
        chart->addAxis(axisy, Qt::AlignLeft); 
        chart->addAxis(axisx, Qt::AlignBottom);
        chart->addSeries(seriesx);  
        chart->addSeries(seriesy);
        chart->addSeries(seriesz);
        chart->setAnimationOptions(QChart::SeriesAnimations); 

    }
    void VR_interaLsys::set_attach(QLineSeries* seriesx,QLineSeries* seriesy,QLineSeries* seriesz,QValueAxis *axis,QValueAxis *axisy){
        seriesx->attachAxis(axis);
        seriesx->attachAxis(axisy);
        seriesy->attachAxis(axis);                             
        seriesy->attachAxis(axisy);
        seriesz->attachAxis(axis);                             
        seriesz->attachAxis(axisy);

    }
    
    void VR_interaLsys::uwb_poseA_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg){
        poseA.x=msg->position.x;
        poseA.y=msg->position.y;
        // pointreceieved();
    }
    void VR_interaLsys::uwb_poseB_callback(const geometry_msgs::msg::Pose2D::ConstSharedPtr msg){
        poseB.x=msg->x;
        poseB.y=msg->y;
        // pointreceieved();
    }
    
    void VR_interaLsys::poseA_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
        poseA.x=msg->pose.position.x;
        poseA.y=msg->pose.position.y;
        // pointreceieved();
    }
    void VR_interaLsys::poseB_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg){
        poseB.x=msg->pose.position.x;
        poseB.y=msg->pose.position.y;
        // pointreceieved();
    }
    void VR_interaLsys::jointState_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg){

        ui_->fl_Speed_V->setText(QString::number(msg->velocity[0],'f',2));
        ui_->rl_Speed_V->setText(QString::number(msg->velocity[1],'f',2));
        ui_->rr_Speed_V->setText(QString::number(msg->velocity[2],'f',2));
        ui_->fr_Speed_V->setText(QString::number(msg->velocity[3],'f',2));

    }
    
    void VR_interaLsys::cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
        ui_->actual_lineal_speed->setText(QString::number(msg->linear.x,'f',2));
        ui_->actual_angual_speed->setText(QString::number(msg->angular.z,'f',2));
    }
    void VR_interaLsys::cmd_vel_callback_B(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
        ui_->actual_lineal_speed_B->setText(QString::number(msg->linear.x,'f',2));
        ui_->actual_angual_speed_B->setText(QString::number(msg->angular.z,'f',2));
    }
    void VR_interaLsys::cmd_vel_callback_V(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
        ui_->actual_lineal_speed_V->setText(QString::number(msg->linear.x,'f',2));
        ui_->actual_angual_speed_V->setText(QString::number(msg->angular.z,'f',2));
    }

    void VR_interaLsys::speedA(const nav_msgs::msg::Odometry::ConstSharedPtr msg){
        wheel_npm A;
        
        A.fl=msg->twist.twist.linear.x-msg->twist.twist.linear.z*(car_setting::Wheel_spacing+car_setting::Wheel_axlespacing);
        A.fr=msg->twist.twist.linear.x+msg->twist.twist.linear.z*(car_setting::Wheel_spacing+car_setting::Wheel_axlespacing);
        A.rr=msg->twist.twist.linear.x+msg->twist.twist.linear.z*(car_setting::Wheel_spacing+car_setting::Wheel_axlespacing);
        A.rl=msg->twist.twist.linear.x-msg->twist.twist.linear.z*(car_setting::Wheel_spacing+car_setting::Wheel_axlespacing);
        ui_->fl_Speed->setText(QString::number(A.fl,'f',2));
        ui_->fr_Speed->setText(QString::number(A.fr,'f',2));
        ui_->rl_Speed->setText(QString::number(A.rl,'f',2));
        ui_->rr_Speed->setText(QString::number(A.rr,'f',2));
        
    }
    void VR_interaLsys::speedB(const nav_msgs::msg::Odometry::ConstSharedPtr msg){
        wheel_npm B;
        float R=msg->twist.twist.linear.x/msg->twist.twist.linear.z;
        
        B.fl=msg->twist.twist.linear.x*cos(atan(car_setting::Wheel_axlespacing_B*R));
        B.fr=msg->twist.twist.linear.x*cos(atan(car_setting::Wheel_axlespacing_B*R));
        B.rr=msg->twist.twist.linear.x*(R-0.5*car_setting::Wheel_spacing_B)/R;
        B.rl=msg->twist.twist.linear.x*(R+0.5*car_setting::Wheel_spacing_B)/R;
        ui_->fl_Speed_B->setText(QString::number(B.fl,'f',2));
        ui_->fr_Speed_B->setText(QString::number(B.fr,'f',2));
        ui_->rl_Speed_B->setText(QString::number(B.rl,'f',2));
        ui_->rr_Speed_B->setText(QString::number(B.rr,'f',2));
    }

    
    void VR_interaLsys::set_pushbutton(QPushButton *pushbutton){
        pushbutton->setAutoRepeat(true); //启用长按
        pushbutton->setAutoRepeatDelay(0);//触发长按的时间
        pushbutton->setAutoRepeatInterval(0);//长按时click信号间隔
    }

    void VR_interaLsys::vedit_button_clicked()
    {
        car_setting::RemoteLeftSpeed=(ui_->v_linedit->text()).toFloat();
        car_setting::RemoteLeftAngularSpeed=(ui_->w_linedit->text()).toFloat();
        
    } 
    void VR_interaLsys::vedit_button_clicked_B()
    {
        car_setting::RemoteLeftSpeed_B=(ui_->v_linedit_B->text()).toFloat();
        car_setting::RemoteLeftAngularSpeed_B=(ui_->w_linedit_B->text()).toFloat();
        
    } 
    void VR_interaLsys::vedit_button_clicked_V()
    {
        car_setting::RemoteLeftSpeed_V=(ui_->v_linedit_V->text()).toFloat();
        car_setting::RemoteLeftAngularSpeed_V=(ui_->w_linedit_V->text()).toFloat();
        
    } 
        
    void VR_interaLsys::up_button_clicked()
    {
        std::cout<<"1111"<<std::endl;
        
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.up_(remote_control_msg);
        remote_control_pub_A->publish(remote_control_msg);
        QString text = "前进";
        ui_->mode->setText(text);

    }
    void VR_interaLsys::down_button_clicked()
    {

        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.down_(remote_control_msg);
        remote_control_pub_A->publish(remote_control_msg);
        QString text = "后退";
        ui_->mode->setText(text);


    }
    void VR_interaLsys::left_button_clicked()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.left_(remote_control_msg);
        remote_control_pub_A->publish(remote_control_msg);
        QString text = "左转";
        ui_->mode->setText(text);
    }
    void VR_interaLsys::right_button_clicked()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.right_(remote_control_msg);
        remote_control_pub_A->publish(remote_control_msg);
        QString text = "右转";
        ui_->mode->setText(text);
    }
    void VR_interaLsys::stop_button_clicked()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.stop_(remote_control_msg);
        remote_control_pub_A->publish(remote_control_msg);
        QString text = "停止";
        ui_->mode->setText(text);
    }
    
    void VR_interaLsys::up_button_clicked_B()
    {
        
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.up_B(remote_control_msg);
        remote_control_pub_B->publish(remote_control_msg);
        QString text = "前进";
        ui_->mode_B->setText(text);

    }
    void VR_interaLsys::down_button_clicked_B()
    {

        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.down_B(remote_control_msg);
        remote_control_pub_B->publish(remote_control_msg);
        QString text = "后退";
        ui_->mode_B->setText(text);


    }
    void VR_interaLsys::left_button_clicked_B()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.left_B(remote_control_msg);
        remote_control_pub_B->publish(remote_control_msg);
        QString text = "左转";
        ui_->mode_B->setText(text);
    }
    void VR_interaLsys::right_button_clicked_B()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.right_B(remote_control_msg);
        remote_control_pub_B->publish(remote_control_msg);
        QString text = "右转";
        ui_->mode_B->setText(text);
    }
    void VR_interaLsys::stop_button_clicked_B()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.stop_B(remote_control_msg);
        remote_control_pub_B->publish(remote_control_msg);
        QString text = "停止";
        ui_->mode_B->setText(text);
    }
    
    void VR_interaLsys::up_button_clicked_V()
    {
        
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.up_V(remote_control_msg);
        remote_control_pub_V->publish(remote_control_msg);
        QString text = "前进";
        ui_->mode_V->setText(text);

    }
    void VR_interaLsys::down_button_clicked_V()
    {

        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.left_V(remote_control_msg);
        remote_control_pub_V->publish(remote_control_msg);
        QString text = "后退";
        ui_->mode_V->setText(text);


    }
    void VR_interaLsys::left_button_clicked_V()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.down_V(remote_control_msg);
        remote_control_pub_V->publish(remote_control_msg);
        QString text = "左转";
        ui_->mode_V->setText(text);
    }
    void VR_interaLsys::right_button_clicked_V()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.right_V(remote_control_msg);
        remote_control_pub_V->publish(remote_control_msg);
        QString text = "右转";
        ui_->mode_V->setText(text);
    }
    void VR_interaLsys::stop_button_clicked_V()
    {
        auto remote_control_msg = geometry_msgs::msg::Twist();
        VR_interaLsys button;
        button.stop_V(remote_control_msg);
        remote_control_pub_V->publish(remote_control_msg);
        QString text = "停止";
        ui_->mode_V->setText(text);
    }
    
    void VR_interaLsys::reset_button_clicked(){
        car_setting::RemoteLeftSpeed=0.15;
        car_setting::RemoteLeftAngularSpeed=0.7;

        car_setting::RemoteLeftSpeed_B=0.15;
        car_setting::RemoteLeftAngularSpeed_B=0.7;

        car_setting::RemoteLeftSpeed_V=0.15;
        car_setting::RemoteLeftAngularSpeed_V=0.7;
    }
    
    void VR_interaLsys::controller_parm_button_clicked(){
        k=(ui_->k_->text()).toFloat();
        lfc=(ui_->Lfc->text().toFloat());
        max_speed=(ui_->max_speed->text().toFloat());
        max_omega=(ui_->max_omege->text().toFloat());

        std::ostringstream oss;
        oss << "k = " << k << ", Lfc = " << lfc << ", max_speed = " << max_speed << ", max_omega = " << max_omega;

        std_msgs::msg::String par_msg;
        par_msg.data = oss.str();
        par_V->publish(par_msg);
    }
    void VR_interaLsys::points_cleart_button_clicked(){
        points.clear();
        points_B.clear();
    }
 
    void VR_interaLsys::slotTimeout_test()
    {
        if(pointCount > AXIS_MAX_Y)
        {
            Acc.x->remove(0);
            Acc.y->remove(0);
            Acc.z->remove(0);
            Angual.x->remove(0);
            Angual.y->remove(0);
            Angual.z->remove(0);
            Mag.x->remove(0);
            Mag.y->remove(0);
            Mag.z->remove(0);

            Acc.chart->axisX()->setMin(pointCount - AXIS_MAX_Y);
            Acc.chart->axisX()->setMax(pointCount);                    // 更新X轴范围
            Angual.chart->axisX()->setMin(pointCount - AXIS_MAX_Y);
            Angual.chart->axisX()->setMax(pointCount);                    // 更新X轴范围
            Mag.chart->axisX()->setMin(pointCount - AXIS_MAX_Y);
            Mag.chart->axisX()->setMax(pointCount);                    // 更新X轴范围

        }
        
        Acc.x->append(QPointF(pointCount, imu_B.Acc.x));
        Acc.y->append(QPointF(pointCount,imu_B.Acc.y));
        Acc.z->append(QPointF(pointCount, imu_B.Acc.z));

        Angual.x->append(QPointF(pointCount, imu_B.Angual.x));  
        Angual.y->append(QPointF(pointCount,imu_B.Angual.y));
        Angual.z->append(QPointF(pointCount, imu_B.Angual.z));

        Mag.x->append(QPointF(pointCount, imu_B.Mag.x));
        Mag.y->append(QPointF(pointCount, imu_B.Mag.y));
        Mag.z->append(QPointF(pointCount, imu_B.Mag.z));

        pointCount++;
    }
    
    void VR_interaLsys::slotTimeout_test_B()
    {
        if(pointCount_B > AXIS_MAX_Y)
        {
            Acc_B.x->remove(0);
            Acc_B.y->remove(0);
            Acc_B.z->remove(0);
            Angual_B.x->remove(0);
            Angual_B.y->remove(0);
            Angual_B.z->remove(0);
            Mag_B.x->remove(0);
            Mag_B.y->remove(0);
            Mag_B.z->remove(0);

            Acc_B.chart->axisX()->setMin(pointCount_B - AXIS_MAX_Y);
            Acc_B.chart->axisX()->setMax(pointCount_B);                    // 更新X轴范围
            Angual_B.chart->axisX()->setMin(pointCount_B - AXIS_MAX_Y);
            Angual_B.chart->axisX()->setMax(pointCount_B);                    // 更新X轴范围
            Mag_B.chart->axisX()->setMin(pointCount_B - AXIS_MAX_Y);
            Mag_B.chart->axisX()->setMax(pointCount_B);                    // 更新X轴范围

        }
        
        Acc_B.x->append(QPointF(pointCount_B, imu_B.Acc.x));
        Acc_B.y->append(QPointF(pointCount_B,imu_B.Acc.y));
        Acc_B.z->append(QPointF(pointCount_B, imu_B.Acc.z));

        Angual_B.x->append(QPointF(pointCount_B, imu_B.Angual.x));  
        Angual_B.y->append(QPointF(pointCount_B,imu_B.Angual.y));
        Angual_B.z->append(QPointF(pointCount_B, imu_B.Angual.z));

        Mag_B.x->append(QPointF(pointCount_B, imu_B.Mag.x));
        Mag_B.y->append(QPointF(pointCount_B, imu_B.Mag.y));
        Mag_B.z->append(QPointF(pointCount_B, imu_B.Mag.z));

        pointCount_B++;
    }
    
    QImage VR_interaLsys::convertToQImage(const cv::Mat& image) {
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
    
    sensors_msg::ImageProto VR_interaLsys::deserializeImage(const std_msgs::msg::ByteMultiArray::ConstSharedPtr& ser_msg){
        sensors_msg::ImageProto msg_pb;
        const auto pb_start_t = std::chrono::high_resolution_clock::now();

        std::vector<uint8_t> msg_data(ser_msg->data.begin(), ser_msg->data.end());
        msg_pb.ParseFromArray(msg_data.data(), msg_data.size());

        return msg_pb;
    }
    cv::Mat VR_interaLsys::convertToCVMat(const sensors_msg::ImageProto& msg_pb) {
        int height = msg_pb.height();
        int width = msg_pb.width();
        int channels = msg_pb.step() / msg_pb.width();
        cv::Mat image_rgb8(height, width, CV_8UC(channels));
        memcpy(image_rgb8.data, msg_pb.data().c_str(), msg_pb.data().size());

        return image_rgb8;
    }
    
    pointcloud2::pointCloud2_pb VR_interaLsys::deserializePointCloud2(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg) {
        std::cout<<"ser_size"<<sizeof(ser_msg)<<std::endl;
        
        pointcloud2::pointCloud2_pb msg_pb;
        std::vector<uint8_t> msg_data(ser_msg->data.begin(), ser_msg->data.end()); 
        msg_pb.ParseFromArray(msg_data.data(), msg_data.size());
        return msg_pb;
    }
    sensor_msgs::msg::PointCloud2 VR_interaLsys::convertPointCloud2(const pointcloud2::pointCloud2_pb& msg_pb) {
        std::cout<<"pb_size"<<sizeof(msg_pb)<<std::endl;
        sensor_msgs::msg::PointCloud2 cloud;
        using PointField = sensor_msgs::msg::PointField;   
        cloud.height = 1;
        cloud.width = 0;
        cloud.fields.resize(6);
        cloud.fields[0].offset = 0;
        cloud.fields[0].name = "x";
        cloud.fields[0].count = 1;
        cloud.fields[0].datatype = PointField::FLOAT32;
        cloud.fields[1].offset = 4;
        cloud.fields[1].name = "y";
        cloud.fields[1].count = 1;
        cloud.fields[1].datatype = PointField::FLOAT32;
        cloud.fields[2].offset = 8;
        cloud.fields[2].name = "z";
        cloud.fields[2].count = 1;
        cloud.fields[2].datatype = PointField::FLOAT32;
        cloud.fields[3].offset = 12;
        cloud.fields[3].name = "intensity";
        cloud.fields[3].count = 1;
        cloud.fields[3].datatype = PointField::FLOAT32;
        cloud.fields[4].offset = 16;
        cloud.fields[4].name = "tag";
        cloud.fields[4].count = 1;
        cloud.fields[4].datatype = PointField::UINT8;
        cloud.fields[5].offset = 17;
        cloud.fields[5].name = "line";
        cloud.fields[5].count = 1;
        cloud.fields[5].datatype = PointField::UINT8;
        cloud.point_step = msg_pb.point_step();
        cloud.width=msg_pb.width();
        cloud.row_step=msg_pb.row_step();
        cloud.header.frame_id=msg_pb.frame_id();
        cloud.header.stamp.nanosec=msg_pb.stamp_nanosec();
        cloud.header.stamp.sec=msg_pb.stamp_sec();
        cloud.is_bigendian=msg_pb.is_bigendian();
        cloud.is_dense=msg_pb.is_dense();

        cloud.data.resize(msg_pb.data().size());
        memcpy(cloud.data.data(),msg_pb.data().data(),msg_pb.data().size());

        // 将转换后的数据拷贝到 cloud.data 中
        
        return cloud;
    }
    
    void VR_interaLsys::up_(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x=1* car_setting::RemoteForwardSpeed;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::down_(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x= -1* car_setting::RemoteForwardSpeed;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::left_(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=1* car_setting::RemoteLeftAngularSpeed;
        msg.linear.x= 1* car_setting::RemoteLeftSpeed;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::right_(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=-1* car_setting::RemoteLeftAngularSpeed;
        msg.linear.x=1* car_setting::RemoteLeftSpeed;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::stop_(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x=0;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    
    void VR_interaLsys::up_B(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x=1* car_setting::RemoteForwardSpeed_B;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::down_B(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x= -1* car_setting::RemoteForwardSpeed_B;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::left_B(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=1* car_setting::RemoteLeftAngularSpeed_B;
        msg.linear.x= 1* car_setting::RemoteLeftSpeed_B;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::right_B(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=-1* car_setting::RemoteLeftAngularSpeed_B;
        msg.linear.x=1* car_setting::RemoteLeftSpeed_B;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::stop_B(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x=0;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    
    void VR_interaLsys::up_V(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x=1* car_setting::RemoteForwardSpeed_V;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::down_V(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x= -1* car_setting::RemoteForwardSpeed_V;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::left_V(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=1* car_setting::RemoteLeftAngularSpeed_V;
        msg.linear.x= 1* car_setting::RemoteLeftSpeed_V;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::right_V(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=-1* car_setting::RemoteLeftAngularSpeed_V;
        msg.linear.x=1* car_setting::RemoteLeftSpeed_V;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    void VR_interaLsys::stop_V(geometry_msgs::msg::Twist& msg){
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;
        msg.linear.x=0;
        msg.linear.y=0;
        msg.linear.z=0;
    }
    
    double VR_interaLsys::medianfilter(std::deque<double> frequency_){
        std::vector<double> sort_value(frequency_.begin(),frequency_.end());
        std::sort(sort_value.begin(),sort_value.end());
        size_t size=sort_value.size();
        double median;
        if(frequency_.size()%2==0){
            median=(frequency_[frequency_.size()/2 -1]+frequency_[frequency_.size()/2])/2.0;
        }
        else{
            median=frequency_[frequency_.size()/2];
        }
        return median;
    }

} // namespace rqt_plugin