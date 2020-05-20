
cmake_minimum_required(VERSION 3.5)
set (CMAKE_PREFIX_PATH "/opt/ros/kinetic")
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set (CMAKE_CXX_STANDARD 14)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()


find_package(gtest)


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(ORCAStarLib)

include_directories(./ORCAStarLib/Src)


rosbuild_genmsg()
rosbuild_gensrv()


rosbuild_add_executable(ROSSimActor src/StartActor.cpp src/ROSSimActor.h src/ROSSimActor.cpp)
rosbuild_add_executable(ROSMission src/StartMission.cpp src/ROSMission.h src/ROSMission.cpp)
rosbuild_add_executable(ROSAgent src/StartAgent.cpp src/ROSAgent.h src/ROSAgent.cpp)
rosbuild_add_executable(ROSVisualization src/StartVisualization.cpp)




target_link_libraries(ROSSimActor ORCAStarLib TINYXML)
target_link_libraries(ROSMission ORCAStarLib TINYXML)
target_link_libraries(ROSAgent ORCAStarLib TINYXML)


file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false ./test/*.h
                                                      ./test/*.cpp)

rosbuild_add_executable(test/ORCAStarTest ${TEST_SOURCES})

rosbuild_add_gtest_build_flags(test/ORCAStarTest)

target_link_libraries(test/ORCAStarTest ORCAStarLib TINYXML)

