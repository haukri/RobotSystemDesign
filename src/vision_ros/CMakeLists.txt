cmake_minimum_required(VERSION 2.8.3)
project(vision_ros)

set(OpenCV_DIR "/usr/share/OpenCV")

find_package(catkin REQUIRED
  roscpp
  image_transport
  cv_bridge
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp
  DEPENDS

)

find_package(Opencv REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(vision_ros src/vision_ros)
add_executable(seg_ros src/seg_ros)
add_executable(image_pub src/image_pub)
add_executable(cam_test src/cam_test)

target_link_libraries(vision_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(seg_ros
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(image_pub
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(cam_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
