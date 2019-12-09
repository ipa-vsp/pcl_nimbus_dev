cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(visualiztion_test)

find_package(PCL 1.9 REQUIRED COMPONENTS common io visualization features console)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main src/main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})
