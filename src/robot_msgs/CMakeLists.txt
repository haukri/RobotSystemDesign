cmake_minimum_required(VERSION 2.8.3)
project(robot_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(
  DIRECTORY srv
  FILES
    RobotCommand.srv
)

add_message_files(
  DIRECTORY msg
  FILES
    RobotCmd.msg
    RobotStatus.msg
)


generate_messages(
  DEPENDENCIES
    std_msgs
)


catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
