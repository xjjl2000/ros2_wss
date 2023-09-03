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

find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (geometry_msgs REQUIRED)
find_package(ZLIB REQUIRED)
find_package (rosidl_default_generators REQUIRED)



add_executable (listener src/convert.cpp )
# 添加可执行文件所需的头文件路径
set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include ${OpenCV_INCLUDE_DIRS})

target_include_directories(listener PRIVATE include ${OpenCV_INCLUDE_DIRS})

set(OpenCV_DIR /usr/local/share/OpenCV)
message(STATUS "version: ${OpenCV_VERSION}")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
)
target_sources (listener PRIVATE 
                        src/callback/CompentsCallback.cpp
                        src/callback/ConnectCallback.cpp
                        src/callback/DeliveryComplete.cpp
                        src/callback/CompentsActionCallback.cpp
                        src/callback/MessageArrivedCallback.cpp
                        src/callback/ActionCallback.cpp
                        src/callback/CallbackFunctionInterface.cpp
                        src/MyMqttClient.cpp
                        )
# target_sources (listener PRIVATE src/mqtt_sub.cpp)


# add_library (listener_lib 
#             src/mqtt_pub.cpp
#             src/mqtt_sub.cpp)
# target_include_directories(listener_lib PRIVATE include)


list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Mosquitto REQUIRED)
find_package(eclipse-paho-mqtt-c REQUIRED)
link_directories(/usr/local/lib)
include_directories(${MOSQUITTO_INCLUDE_DIR})

set(PROTO_PB_DIR ${PROJECT_SOURCE_DIR}/third/sensor_protobuf)
include_directories(${PROJECT_SOURCE_DIR}
        ${PROTO_PB_DIR}
        ${ZLIB_INCLUDE_DIRS})
add_subdirectory(third/sensor_protobuf)
target_link_libraries(listener _pb3 mosquitto ${ZLIB_LIBRARIES} paho-mqttpp3 paho-mqtt3a)



ament_target_dependencies (listener rclcpp sensor_msgs qt_gui_cpp cv_bridge) 

install (TARGETS listener DESTINATION lib/${PROJECT_NAME})

ament_package()