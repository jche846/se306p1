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

rosbuild_add_executable(supervisor
  src/supervisor/supervisor.cpp
  src/supervisor/robot.cpp
  src/supervisor/behaviors/rotate_behavior.cpp
  src/supervisor/behaviors/goto_circle_behavior.cpp
  src/supervisor/behaviors/goto_triangle_behavior.cpp
  src/supervisor/behaviors/goto_square_behavior.cpp
  src/util/trig.cpp)
set_target_properties(supervisor PROPERTIES COMPILE_DEFINITIONS SUPERVISOR_MAIN)

rosbuild_add_executable(robot_controller
  src/controller/robot_controller.cpp
  src/util/trig.cpp)
set_target_properties(robot_controller PROPERTIES COMPILE_DEFINITIONS ROBOT_CONTROLLER_MAIN)

rosbuild_add_gtest(test_command
    test/command.cpp)
rosbuild_add_gtest(test_trig
    src/util/trig.cpp
    test/trig.cpp)
rosbuild_add_gtest(test_supervisor
    src/util/trig.cpp
    test/supervisor.cpp)
rosbuild_add_gtest(test_pose
    test/pose.cpp)
rosbuild_add_gtest(test_vector2
    test/vector2.cpp)
rosbuild_add_gtest(test_robot_controller
    test/robot_controller.cpp
    src/util/trig.cpp
    src/controller/robot_controller.cpp)
rosbuild_add_gtest(test_supervisor
    test/supervisor.cpp)
rosbuild_add_rostest(test/supervisor.test)

#target_link_libraries(example ${PROJECT_NAME})
