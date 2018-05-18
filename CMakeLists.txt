cmake_minimum_required(VERSION 2.8.3)
project(GMM)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  genmsg 
  actionlib_msgs 
  actionlib
)


find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_action_files(DIRECTORY action FILES GMM.action)
generate_messages(DEPENDENCIES actionlib_msgs)
catkin_package(
  CATKIN_DEPENDS cv_bridge image_transport roscpp std_msgs sensor_msgs
)



add_executable(gmm_r3 src/gmm_r3.cpp)
target_link_libraries(gmm_r3 ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(pub_r src/publisher_r.cpp)
target_link_libraries(pub_r  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})

add_executable(sub_r src/subscriber_r.cpp)
target_link_libraries(sub_r  ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
add_executable(kalman_r src/kalman_r.cpp)
target_link_libraries(kalman_r ${OpenCV_LIBRARIES}  ${catkin_LIBRARIES})
