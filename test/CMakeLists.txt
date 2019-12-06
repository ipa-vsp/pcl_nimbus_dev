cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(test)

find_package(PCL 1.9 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test src/pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})