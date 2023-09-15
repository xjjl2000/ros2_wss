#ifndef PUSH_BUTTON_H__
#define PUSH_BUTTON_H__

// ROS2
#include <rclcpp/rclcpp.hpp>

// msgs
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"


// Rqt
#include <rqt_gui_cpp/plugin.h>

// Qt
#include <QWidget>
#include <QImage>
#include <QTimer>
#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QChartView>
#include <QTableWidget>
#include <QPaintEvent> //用于绘画事件

// Custom UI
#include <my_plugin/ui_rqt_push_button.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "sensor_msgs/image_encodings.hpp"
#include <image_transport/camera_subscriber.hpp>
#include "image_transport/subscriber.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>

//car_control
#include <my_plugin/car_init.hpp>
#include <my_plugin/camera.hpp>
#include <my_plugin/cmd_speed.hpp>




#include "std_msgs/msg/byte_multi_array.hpp"
#include "my_interface/msg/image_compressed.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "sensors_msg.pb.h"

#include "pointcloud2.pb.h"
#include "rviz_common/panel.hpp"
#include "rviz_common/display.hpp"

#include "MyMqttClient.hpp"
#include <QMutex>

#include "vtkRenderWindow.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>


QT_CHARTS_USE_NAMESPACE
namespace Ui {
class myWidget;
}

namespace rviz2_plugin
{
  class imuchart{
    public:
      QValueAxis* ho;
      QValueAxis*lo;
      QLineSeries* x;
      QLineSeries*y;
      QLineSeries*z;
      QChart* chart;
      imuchart(){
        ho=new QValueAxis();
        lo=new QValueAxis();
        x=new QLineSeries();
        y=new QLineSeries();
        z=new QLineSeries();
        chart=new QChart();
      }
      ~imuchart(){
        delete ho;
        delete lo;
        delete x;
        delete y;
        delete z;
        delete chart;
      }
  };
  class VR_interaLsys : public rviz_common::Panel
  {
    Q_OBJECT
  public:
    VR_interaLsys(QWidget* parent = nullptr);

    ~VR_interaLsys() override;

    void onInitialize() override;
    void onEnable();
    void onDisable();


    



  protected:

    

    MyMqttClient mqtt_image_sub;
    MyMqttClient mqtt_cmd_sub;
    MyMqttClient mqtt_imu_sub;


    MyMqttClient mqtt_cmd_pub;
    Ui::myWidget* ui_;
    QMutex qMtuxLabel1,qMtuxLabel2;
    QWidget *widget_;
    QVector<QPointF> points;
    QVector<QPointF> points_B;
    
    size_t hzt,hhzt;

    std::thread spin_thread_;

    bool running_=true;
    static bool alive;
    static bool connected ;
    

    /* 用于模拟生成实时数据的定时器 */
    QTimer *CurrentTime;
    struct timespec lastTime = {0, 0};
    struct timespec lastTime1 = {0, 0};
    struct timespec lastTime2 = {0, 0};
    struct timespec lastTime3 = {0, 0};


    /* 图表对象 */

    QTabWidget *A_tabWidget,*B_tabWidget;


    QFont labelsFont;


    /* 横纵坐标最大显示范围 */
    const int AXIS_MAX_X = 6, AXIS_MAX_Y = 10;
    const int MAG_AXIS_MAX_X = 6, MAG_AXIS_MAX_Y = 2;

    /* 用来记录数据点数 */

    int pointCount = 0;int pointCount_B=0;
    double k,lfc,max_speed,max_omega;

    int w=360,h=360;//画布大小
    int pointx=20,pointy=h-10;//确定坐标轴起点坐标，这里定义(35,280)
    int width=w-pointx,height=h-20;//确定坐标轴宽度跟高度 上文定义画布为371*361，宽高依此而定。

    int cloud_count,rearimage_count,rearimage_pb_count;


    typedef struct{
      int nanosec;
      int sec;
    }stamp_t;

    typedef struct{
      struct timespec image_f;
      struct timespec image_b;
      struct timespec imu;
      struct timespec pointcloud2;
    }lasttime;

    typedef struct{
      double x;
      double y;
    }location;

    typedef struct{
      float x;
      float y;
      float z; 
    }xyz;
    typedef struct{
      xyz Acc;
      xyz Angual;
      xyz Mag;
    }IMU_msg;

    typedef struct{
      double imgf_hz;
      double imgb_hz;
      double poindcloud2_hz;
      double imu_hz;
    }medianHz;

    std::deque<double> frequency_value;
    std::deque<double> freq_imgb_B;
    std::deque<double> freq_imu;



    location poseA,poseB;

    stamp_t imu_At,imu_Bt,
            image_At_f,image_At_b,image_Bt_f,image_Bt_b,image_Vt_f,image_Vt_b,
            cloud_At,cloud_Bt;

    IMU_msg imu_A,imu_B;

    imuchart Acc,Angual,Mag;
    imuchart Acc_B,Angual_B,Mag_B;

    lasttime A,B;
    std::vector<timespec> intervals;
    


    typedef struct{
      double fl;
      double fr;
      double rl;
      double rr;
    }wheel_npm;
    
    typedef struct {
      float x;            /**< X axis, Unit:m */
      float y;            /**< Y axis, Unit:m */
      float z;            /**< Z axis, Unit:m */
      float reflectivity; /**< Reflectivity   */
      uint8_t tag;        /**< Livox point tag   */
      uint8_t line;       /**< Laser line id     */
    } LivoxPointXyzrtl;

    typedef struct {
      float x;
      float y;
      float z;
      float intensity;
      uint8_t tag;
      uint8_t line;
      uint64_t offset_time;
    } PointXyzlt;

    
    Cmd_Speed cmd_;

  protected:
    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor executor_;


    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud ;
    pcl::visualization::PCLVisualizer::Ptr view;
    uint8_t r, g, b;
    
    //图像
    //前置
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_sub_A;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_compressed_sub_B;

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr image_f_pb_A;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr image_f_pb_B;

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr Compressedimage_f_pb_A;

    
    //后置
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_back_compressed_sub_A;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_back_compressed_sub_B;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_back_compressed_sub_V;
    
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr image_r_pb_B;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr image_r_pb_A;

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr image_back_pb_V;


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_raw_V;

    //图像序列化

    //远程控制
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_control_pub_A;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_control_pub_B;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr remote_control_pub_V;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointstate_V;

    //april_tag
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr point;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr point_B;

    //uwb
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr point_uwb;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr point_ubw_B;


    //实际线性速度、角速度订阅
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_A;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_B;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_V;

    //实际轮速订阅
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nmp_A;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nmp_B;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr nmp_V;

    //IMU
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr actualDataLine_sub_A;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr actualMag_data_sub_A;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr actualDataLine_sub_B;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr actualMag_data_sub_B;

    //点云
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_A;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_sub_B;

    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr pointcloud2pb_sub_A;
    rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr pointcloud2pb_sub_B;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser;




    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pb_2_cloud2_A;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pb_2_cloud2_B;


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr par_V;
    


    cv_bridge::CvImagePtr cv_ptr;
  
  signals:
    void updateLabel(QLabel* label,QPixmap & data);


    
 




  public slots:
    

  protected slots:


    void spin();
    //绘制定位点
    bool eventFilter(QObject *watched, QEvent *event) ;
    void paintEvent();
    void paintEventB();

    //初始化按钮
    void set_pushbutton(QPushButton *pushbutton);

    //IMU_chart
    void set_tabWidget_test(QTabWidget *widget,imuchart& imu_chart,imuchart& imu_chart1,imuchart& imu_chart2,QLabel *chartlabel);

    void set_tabWidget(QTabWidget *widget,QLabel *chartlabel);
    void set_tabWidget_B(QTabWidget *widget,QLabel *chartlabel);

    void set_axis(QValueAxis *axisx,QValueAxis *axisy);
    void set_series(QLineSeries* seriesx,QLineSeries* seriesy,QLineSeries* seriesz);
    void set_chart(QChart* chart,QValueAxis *axisx,QValueAxis *axisy,QLineSeries* seriesx,QLineSeries* seriesy,QLineSeries* seriesz);
    void set_attach(QLineSeries* seriesx,QLineSeries* seriesy,QLineSeries* seriesz,QValueAxis *axis,QValueAxis *axisy);

    QImage convertToQImage(const cv::Mat& image);//senosor_msg转化成图片

    //imageraw_deserialize
    sensors_msg::ImageProto deserializeImage(const std_msgs::msg::ByteMultiArray::ConstSharedPtr& ser_msg);
    cv::Mat convertToCVMat(const sensors_msg::ImageProto& msg_pb);

    //pointcloud_deserialize
    pointcloud2::pointCloud2_pb deserializePointCloud2(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg);
    sensor_msgs::msg::PointCloud2 convertPointCloud2(const pointcloud2::pointCloud2_pb& msg_pb);


    void V_back_image_raw_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
    void A_front_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
    void B_front_imagecallback(sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
    void A_back_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);
    void B_back_compressed_imagecallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg);

    //deserialize
    void Af_ser_compressedimagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg);

    void Af_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg);
    void Bf_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg);
    void Ar_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg);
    void Br_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg);
    void V_ser_imagecallback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr ser_msg);


    void CameraImageRawCallback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg,const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
    void imagecallback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);

    void uwb_poseA_callback(const geometry_msgs::msg::Pose::ConstSharedPtr msg);
    void uwb_poseB_callback(const geometry_msgs::msg::Pose2D::ConstSharedPtr msg);
    
    void poseA_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
    void poseB_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);

    void jointState_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);

    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void cmd_vel_callback_B(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void cmd_vel_callback_V(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

    void speedA(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void speedB(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void speedV(const nav_msgs::msg::Odometry::ConstSharedPtr msg);




    void imudata_callback(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line);
    void imumag_callback(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line);
    void imudata_callback_B(const sensor_msgs::msg::Imu::ConstSharedPtr actualData_line);
    void imumag_callback_B(const sensor_msgs::msg::MagneticField::ConstSharedPtr actualMagData_line);

    void pointcloud2_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud2_msg);
    void pointcloud2pb_callback(const std_msgs::msg::ByteMultiArray::ConstSharedPtr pointcloud2pb_msg);//未拆开
    void pointcloud2pb_callback_A(const std_msgs::msg::ByteMultiArray::ConstSharedPtr pointcloud2pb_msg);
    void pointcloud2pb_callback_B(const std_msgs::msg::ByteMultiArray::ConstSharedPtr pointcloud2pb_msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

    // void mqtt_send();
    // void mqtt_receive();
    // void message_callback(struct mosquitto *mosq, void *obj,const struct mosquitto_message *message);
    // void handle_signal(int s);
    // void connect_callback(struct mosquitto *mosq_, void *obj, int result);
    
    void UDP_receive();
    void UDP_send();

    void onTabCloseRequested(int index);
    
    //控制
    void controller_parm_button_clicked();
    
    void vedit_button_clicked();
    void vedit_button_clicked_B();
    void vedit_button_clicked_V();

    void up_button_clicked();
    void down_button_clicked();
    void left_button_clicked();
    void right_button_clicked();
    void stop_button_clicked();

    void up_button_clicked_V();
    void down_button_clicked_V();
    void left_button_clicked_V();
    void right_button_clicked_V();
    void stop_button_clicked_V();

    void up_button_clicked_B();
    void down_button_clicked_B();
    void left_button_clicked_B();
    void right_button_clicked_B();
    void stop_button_clicked_B();
    
    void up_(geometry_msgs::msg::Twist& msg);
    void down_(geometry_msgs::msg::Twist& msg);
    void left_(geometry_msgs::msg::Twist& msg);
    void right_(geometry_msgs::msg::Twist& msg);
    void stop_(geometry_msgs::msg::Twist& msg);

    void up_B(geometry_msgs::msg::Twist& msg);
    void down_B(geometry_msgs::msg::Twist& msg);
    void left_B(geometry_msgs::msg::Twist& msg);
    void right_B(geometry_msgs::msg::Twist& msg);
    void stop_B(geometry_msgs::msg::Twist& msg);

    void up_V(geometry_msgs::msg::Twist& msg);
    void down_V(geometry_msgs::msg::Twist& msg);
    void left_V(geometry_msgs::msg::Twist& msg);
    void right_V(geometry_msgs::msg::Twist& msg);
    void stop_V(geometry_msgs::msg::Twist& msg);
    
    void reset_button_clicked();
    void points_cleart_button_clicked();
    void slotTimeout();
    void slotTimeout_B();
    void slotTimeout_test();
    void slotTimeout_test_B();

    double medianfilter(std::deque<double> frequency_);
    void handlemedian(double freq);

    void addfreq(double hz);
    double getMedian();
    void updateLabelSlot(QLabel* label,QPixmap & data);



  };

  
} // namespace rqt_plugin

#endif // PUSH_BUTTON_H__
