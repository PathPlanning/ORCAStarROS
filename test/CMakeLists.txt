#enable_testing()

#include_directories(../src)
#include_directories(../ORCAStarLib/Src)


#set(BINARY ORCAStarLibTests)

#file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp)

#set(SOURCES ${TEST_SOURCES})

#rosbuild_add_executable(${BINARY} ${TEST_SOURCES})
#rosbuild_add_gtest(${BINARY} MapTest.cpp)
#rosbuild_add_gtest_build_flags(${BINARY})

#target_link_libraries(${BINARY} PUBLIC ORCAStarLib TINYXML gtest )
