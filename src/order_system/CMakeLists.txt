cmake_minimum_required(VERSION 2.8.3)
project(order_system)

find_package(catkin REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

#############
## Install ##
#############

install(PROGRAMS
   scripts/talker.js
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

