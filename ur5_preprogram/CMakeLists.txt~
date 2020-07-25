cmake_minimum_required(VERSION 2.8.3)
project(ur5_preprogram)

find_package(catkin REQUIRED COMPONENTS roslaunch moveit_ros_planning_interface roscpp rospy geometry_msgs)

catkin_package()

#roslaunch_add_file_check(launch)

add_executable(joint_waypoints src/joint_waypoints.cpp)

add_dependencies(joint_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(joint_waypoints ${catkin_LIBRARIES})

#catkin_install_python(PROGRAMS scripts/stow_ur5 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS joint_waypoints RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
