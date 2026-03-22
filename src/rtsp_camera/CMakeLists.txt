cmake_minimum_required(VERSION 3.0.2)
project(rtsp_camera)

## Compile as C++11, supported in ROS Kinetic and newer
# 如果您的包中包含C++代码，并且需要使用C++11，请取消注释以下行
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  tf
  message_filters
)

## System dependencies are found with CMake's conventions
# 如果您的包依赖于系统库（例如Boost），请在这里添加
# find_package(Boost REQUIRED COMPONENTS system)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS cv_bridge rospy sensor_msgs std_msgs tf message_filters
  # 如果您的包包含消息、服务或动作，请在这里添加
  # DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# 如果您的包包含C++库，请取消注释并配置以下内容
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/rtsp_camera.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# 如果您的包包含C++可执行文件，请取消注释并配置以下内容
# add_executable(${PROJECT_NAME}_node src/rtsp_camera_node.cpp)

## Rename C++ executable without prefix
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/rtsp_publisher.py
  scripts/image_display.py
  scripts/imu_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Install additional files (e.g., data, config)
# 如果有其他需要安装的文件夹（如rviz配置文件、数据文件等），请取消注释并添加
# install(DIRECTORY data
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

## Install other resources
# install(FILES
#   # 如果有单独的文件需要安装，可以在这里列出
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

