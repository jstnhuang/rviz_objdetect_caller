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

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(objdetect_caller src/objdetect_caller.cpp)
rosbuild_link_boost(objdetect_caller thread)
rosbuild_add_compile_flags(objdetect_caller "-std=c++0x")

rosbuild_add_executable(objdetect_caller_node src/objdetect_caller_node.cpp)
target_link_libraries(objdetect_caller_node objdetect_caller)
rosbuild_add_compile_flags(objdetect_caller_node "-std=c++0x")

rosbuild_add_library(publish_points src/publish_points.cpp)
rosbuild_link_boost(publish_points thread)
rosbuild_add_compile_flags(publish_points "-std=c++0x")

rosbuild_add_executable(publish_points_node src/publish_points_node.cpp)
target_link_libraries(publish_points_node publish_points)
rosbuild_add_compile_flags(publish_points_node "-std=c++0x")

rosbuild_add_library(publish_boxes src/publish_boxes.cpp)
rosbuild_link_boost(publish_boxes thread)
rosbuild_add_compile_flags(publish_boxes "-std=c++0x")

rosbuild_add_executable(publish_boxes_node src/publish_boxes_node.cpp)
target_link_libraries(publish_boxes_node publish_boxes)
rosbuild_add_compile_flags(publish_boxes_node "-std=c++0x")
