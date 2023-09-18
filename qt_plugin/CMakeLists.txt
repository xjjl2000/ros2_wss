cmake_minimum_required (VERSION 3.5)
project (qt_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options (-Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake REQUIRED)

find_package (rclcpp REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(private_interface REQUIRED)
find_package(Protobuf REQUIRED)
find_package(rosmsg2pbmsg REQUIRED)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5 5.12 COMPONENTS Core Widgets Gui Charts Network REQUIRED)
find_package(Qt5UiTools REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(tf2 REQUIRED)
include_directories(${tf2_INCLUDE_DIRS})
find_package(PCL 1.10 REQUIRED)
find_package(VTK COMPONENTS
  vtkCommonCore
  vtkFiltersSources
  vtkGUISupportQt
  vtkIOImage
  vtkInteractionImage
  vtkFiltersCore
  vtkInfovisCore
  vtkInteractionStyle
)
find_package(pcl_conversions REQUIRED)

include(${VTK_USE_FILE})



set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include 
          ${CMAKE_CURRENT_BINARY_DIR}/..
          ${rclcpp_INCLUDE_DIRS}
          ${std_msgs_INCLUDE_DIRS}
          ${OpenCV_INCLUDE_DIRS}
          ${sensor_msgs_INCLUDE_DIRS}
          ${cv_bridge_INCLUDE_DIRS}
          ${rosmsg2pbmsg_INCLUDE_DIRS}
          ${std_msgs_INCLUDE_DIRS}
          ${Qt5Widgets_INCLUDE_DIRS}
          ${PCL_INCLUDE_DIRS}



)   

set(OpenCV_DIR /usr/local/share/OpenCV)
message(STATUS "version: ${OpenCV_VERSION}")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES})

set(PROTO_PB_DIR ${PROJECT_SOURCE_DIR}/third/sensor_protobuf)
include_directories(${PROJECT_SOURCE_DIR}
        ${PROTO_PB_DIR}
        )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(eclipse-paho-mqtt-c REQUIRED)
link_directories(/usr/local/lib)


add_executable (msgTransit main.cpp)
ament_target_dependencies (msgTransit 
  rclcpp 
  sensor_msgs
  std_msgs
  cv_bridge
  rclcpp_components
  image_transport
  nav_msgs
  rosmsg2pbmsg
  pcl_conversions

)


target_sources (msgTransit PRIVATE  src/RosMessage/imagemsg.cpp
                                    src/RosMessage/pointcloud2msg.cpp
                                    src/RosMessage/imumsg.cpp
                                    src/RosMessage/cmdvelmsg.cpp
                                    src/RosMessage/location.cpp
                                    src/RosMessage/pushButton.cpp
                                    src/RosMessage/wheelSpeed.cpp

                                    src/mqttClient/MessageArrivedCallback.cpp

                                    src/Udpmessage/UdpMessage.cpp
                                    include/handleserver/MyPoint.hpp

                                    src/RosMessage/MainNode.cpp

                                    src/rosserver/RosPointCloud2Callback.cpp
                                    src/rosserver/RosImageCallback.cpp
                                    include/rosserver/RosCallbackInterface.hpp
)

add_subdirectory(third/sensor_protobuf)
target_link_libraries(msgTransit 
      plugin_pb3 
      paho-mqttpp3 
      paho-mqtt3a
      Qt5::Widgets
      Qt5::Core
      Qt5::Gui
      Qt5::UiTools
      Qt5::Network
      ${PCL_LdouIBRARIES} 
      ${VTK_LIBRARIES}
)

install (TARGETS msgTransit DESTINATION lib/${PROJECT_NAME})

ament_package()