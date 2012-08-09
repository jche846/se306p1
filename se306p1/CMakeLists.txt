cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Wfatal-errors -Werror")

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)

rosbuild_add_executable(robot_controller
  src/controller/robot_controller.cpp)

rosbuild_add_executable(rotate_supervisor
  src/supervisor/rotate_supervisor.cpp
  src/supervisor/supervisor.cpp
  src/supervisor/robot.cpp)
set_target_properties(rotate_supervisor PROPERTIES COMPILE_DEFINITIONS ROTATE_SUPERVISOR_MAIN)

rosbuild_add_executable(robot_controller
  src/controller/robot_controller.cpp)

rosbuild_add_gtest(test_command
    test/command.cpp)
rosbuild_add_gtest(test_trig
    test/trig.cpp)
rosbuild_add_gtest(test_supervisor
    test/supervisor.cpp)
rosbuild_add_gtest(test_pose
    test/pose.cpp)
rosbuild_add_gtest(test_vector2
    test/vector2.cpp)
rosbuild_add_gtest(test_robotController
    test/testRobotController.cpp)
rosbuild_add_rostest(test/supervisor.test)

#target_link_libraries(example ${PROJECT_NAME})
