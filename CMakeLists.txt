cmake_minimum_required(VERSION 2.4.6)
project(fydp_demo)
find_package(catkin REQUIRED COMPONENTS std_msgs rospy sensor_msgs geometry_msgs roscpp tf message_generation cv_bridge image_transport)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#uncomment if you have defined services
#rosbuild_gensrv()
add_message_files(DIRECTORY msg FILES ips_msg.msg)
generate_messages(DEPENDENCIES std_msgs)
#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
add_executable(turtlebot_example src/turtlebot_example.cpp src/Map.cpp src/marker.cpp src/local_planner_utils.cpp)
add_executable(sim_pose_publisher src/sim_pose_publisher.cpp)

add_executable(make_a_map src/make_a_map.cpp src/marker.cpp)

#add_executable(cv_demo_flag src/cv_demo_flag.cpp src/FlagDetector.cpp)

#add_executable(cv_demo_lane src/cv_demo_lane.cpp src/cvaux.cpp src/cvcolor.cpp src/cvcontour.cpp src/cvlabel.cpp src/cvtrack.cpp src/LaneDetector.cpp)

add_dependencies(sim_pose_publisher ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(turtlebot_example ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})



#target_link_libraries(turtlebot_example ${PROJECT_NAME})
target_link_libraries(turtlebot_example
   ${catkin_LIBRARIES}
)

target_link_libraries(sim_pose_publisher
   ${catkin_LIBRARIES}
)

target_link_libraries(make_a_map
  ${catkin_LIBRARIES}
)

#target_link_libraries(cv_demo_flag
#  ${catkin_LIBRARIES}
#)

#target_link_libraries(cv_demo_lane
#   ${catkin_LIBRARIES}
#)
