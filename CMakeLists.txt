project(robotont_webapp)


find_package(catkin REQUIRED COMPONENTS
  depthcloud_encoder
  rosbridge_server
  tf2_web_republisher
  web_video_server
)


find_program(NPM npm)
if (NOT NPM)
        message(FATAL_ERROR "npm not found! Follow install instructions on https://github.com/robotont/robotont_webapp")
endif()
message(${NPM})

find_program(TTYD ttyd)
if (NOT TTYD)
        message(FATAL_ERROR "ttyd not found! Follow install instructions on: https://github.com/tsl0922/ttyd#install-on-linux")
endif()
message(${NPM})

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Build frontend and backend
execute_process ( COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/build.sh
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ )

