cmake_minimum_required(VERSION 3.3)

project(webSocket_test)
find_package(PCL 1.9 REQUIRED COMPONENTS common io)
find_package(websocketpp REQUIRED)

find_package(jsoncpp REQUIRED)
#find_package(libjson-rpc-cpp CONFIG REQUIRED)

include_directories(include
                    ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_library(websocket src/websocket.cpp)
#target_link_libraries(websocket ${PCL_LIBRARIES})


add_executable(${PROJECT_NAME} "src/main.cpp")
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
