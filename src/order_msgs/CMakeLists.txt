cmake_minimum_required(VERSION 2.8.3)
project(order_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(
  DIRECTORY srv
  FILES
    NewOrder.srv
)


generate_messages(
  DEPENDENCIES
    std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)