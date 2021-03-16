cmake_minimum_required(VERSION 3.0.2)
project(robotont_webapp)


find_package(catkin REQUIRED COMPONENTS
  depthcloud_encoder
  rosbridge_server
  tf2_web_republisher
  web_video_server
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

