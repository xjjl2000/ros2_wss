cmake_minimum_required (VERSION 3.5)
project (rosmsg2pbmsg)
#set(CMAKE_CXX_STANDARD 14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif()

#set(OpenCV_DIR /usr/local/share/OpenCV)

find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package(ZLIB REQUIRED)
find_package (rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(eclipse-paho-mqtt-c REQUIRED)

message(STATUS "version: ${OpenCV_VERSION}")


# 添加可执行文件所需的头文件路径
set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include 
          ${CMAKE_CURRENT_BINARY_DIR}/..
          ${rclcpp_INCLUDE_DIRS}
          ${std_msgs_INCLUDE_DIRS}
          ${OpenCV_INCLUDE_DIRS}
          ${sensor_msgs_INCLUDE_DIRS}
          ${Qt5Widgets_INCLUDE_DIRS}
          ${qt_gui_cpp_INCLUDE_DIRS}
          ${cv_bridge_INCLUDE_DIRS}
          )



#### 请修改此处
set(my_cv_bridge_INCLUDE_DIRS /home/advrc/ros2_humble/install/cv_bridge/include/cv_bridge)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  ${my_cv_bridge_INCLUDE_DIRS}
)

add_executable (listener src/convert.cpp )
target_include_directories(listener PRIVATE include ${OpenCV_INCLUDE_DIRS})
add_library(MyMqttClient
                        src/callback/CompentsCallback.cpp
                        src/callback/ConnectCallback.cpp
                        src/callback/DeliveryComplete.cpp
                        src/callback/CompentsActionCallback.cpp
                        src/callback/MessageArrivedCallback.cpp
                        src/callback/ActionCallback.cpp
                        src/callback/CallbackFunctionInterface.cpp
                        src/MyMqttClient.cpp
)
set_target_properties(MyMqttClient PROPERTIES
    COMPILE_OPTIONS "-fPIC"
)
# 把库文件复制出目录
install(DIRECTORY include/
  DESTINATION include
)
install(
        TARGETS MyMqttClient
        LIBRARY DESTINATION lib
        )
#从目录中把文件注册到ros2环境空间
ament_export_include_directories(include)
ament_export_libraries(MyMqttClient)

target_sources (listener PRIVATE 
                        src/callback/CompentsCallback.cpp
                        src/callback/ConnectCallback.cpp
                        src/callback/DeliveryComplete.cpp
                        src/callback/CompentsActionCallback.cpp
                        src/callback/MessageArrivedCallback.cpp
                        src/callback/ActionCallback.cpp
                        src/callback/CallbackFunctionInterface.cpp
                        src/MyMqttClient.cpp
                        src/Rosmsg/cloudCallback.cpp
                        src/Rosmsg/imgCallback.cpp
                        src/Rosmsg/imuCallback.cpp

                        )
# target_sources (listener PRIVATE src/mqtt_sub.cpp)


# add_library (listener_lib 
#             src/mqtt_pub.cpp
#             src/mqtt_sub.cpp)
# target_include_directories(listener_lib PRIVATE include)


#list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#find_package(Mosquitto REQUIRED)
link_directories(/usr/local/lib)
#include_directories(${MOSQUITTO_INCLUDE_DIR})

set(PROTO_PB_DIR ${PROJECT_SOURCE_DIR}/third/sensor_protobuf)
include_directories(${PROJECT_SOURCE_DIR}
        ${PROTO_PB_DIR}
        ${ZLIB_INCLUDE_DIRS})
add_subdirectory(third/sensor_protobuf)
  
target_link_libraries(listener _pb3 Qt5::Widgets  ${ZLIB_LIBRARIES} paho-mqttpp3 paho-mqtt3a)



ament_target_dependencies (listener rclcpp 
std_msgs sensor_msgs qt_gui_cpp cv_bridge) 

install (TARGETS listener DESTINATION lib/${PROJECT_NAME})

ament_package()