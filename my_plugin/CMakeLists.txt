cmake_minimum_required(VERSION 3.5)
project(my_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(private_interface REQUIRED) 
find_package(Qt5 5.12 COMPONENTS Core Widgets Gui Charts REQUIRED)
find_package(Qt5UiTools REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Protobuf REQUIRED)
find_package(my_interface REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rviz2 REQUIRED)
find_package(class_loader REQUIRED)
find_package(pluginlib REQUIRED)


#add_subdirectory(src/private_interface)

# Start
# Prepare to create rqt plugin as cpp library

# Define source files
set(${PROJECT_NAME}_SRCS
  src/rviz_push_button.cpp
  src/camera.cpp
  src/cmd_speed.cpp
  src/mqtt_pub.cpp
  src/mqtt_sub.cpp




)

# Define header files
set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/rviz_push_button.hpp
)

# Define ui file
set(${PROJECT_NAME}_UIS
  resource/rqt_push_button.ui
)

# Define include directories
set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${rviz2_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  #${rosmsg2pbmsg_INCLUDE_DIRS}



)
set(OpenCV_DIR /usr/local/share/OpenCV)
message(STATUS "version: ${OpenCV_VERSION}")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Qt wrapper
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})
qt5_wrap_ui(${PROJECT_NAME}_UIS_H ${${PROJECT_NAME}_UIS})

include_directories(
  ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
)
set(PROTO_PB_DIR ${PROJECT_SOURCE_DIR}/third/Image_protobuf)
include_directories(${PROJECT_SOURCE_DIR}
        ${PROTO_PB_DIR}
        )

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Mosquitto REQUIRED)
find_package(eclipse-paho-mqtt-c REQUIRED)
link_directories(/usr/local/lib)
include_directories(${MOSQUITTO_INCLUDE_DIR})

add_library(rviz_push_button
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS}
  ${${PROJECT_NAME}_UIS_H}
  )


ament_target_dependencies(rviz_push_button
  rclcpp
  std_msgs
  qt_gui_cpp
  rqt_gui_cpp
  cv_bridge
  private_interface
  rclcpp_components
  my_interface
  image_transport
  sensor_msgs
  rviz_common
  pluginlib
  rviz2
  nav_msgs
  #rosmsg2pbmsg

)
target_link_libraries(rviz_push_button
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  Qt5::UiTools
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  mosquitto

)

add_subdirectory(third/Image_protobuf)
target_link_libraries(rviz_push_button Qt5::Charts general_img_pb3 paho-mqttpp3 paho-mqtt3a)
# target_compile_options(rviz_push_button PRIVATE -Werror)
# Ends





# # 链接操作
#target_link_libraries(${PROJECT_NAME})
        


install(
  TARGETS rviz_push_button
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include)


install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)






if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

pluginlib_export_plugin_description_file(rviz_common "plugin.xml")


ament_export_include_directories(include)



# ament_export_targets(
#   export_v4l2_camera
# )
#ament_export_libraries(
# v4l2_camera
#)

ament_export_libraries(rviz_push_button )
#ament_export_dependencies(
#  "camera_info_manager"
#)

ament_package()
