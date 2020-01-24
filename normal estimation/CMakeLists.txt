cmake_minimum_required(VERSION 3.0 FATAL_ERROR)

project(normal_estimation)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(PCL 1.9 REQUIRED)
find_package(websocketpp REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#-------- Add Libraries: Web Socket and JSON RPC -----------
add_library(base64 "src/base64.cpp")
target_link_libraries(base64)
add_library(WebSocketClient "src/websocket.cpp")
target_link_libraries(WebSocketClient base64 jsoncpp ${CURL_LIBRARIES} ${WEBSOCKETPP_LIBRARIES})

#-------- Executables ------------
add_executable(main src/main.cpp)
target_link_libraries(main WebSocketClient ${PCL_LIBRARIES})

add_executable(testing src/testing.cpp)
target_link_libraries(testing ${PCL_LIBRARIES})

add_executable(testSeg src/testSeg.cpp)
target_link_libraries(testSeg WebSocketClient ${PCL_LIBRARIES})