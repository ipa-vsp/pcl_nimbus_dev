cmake_minimum_required(VERSION 3.3)

project(webSocket_test)

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

find_package(PCL 1.9 REQUIRED COMPONENTS common io visualization)
find_package(websocketpp REQUIRED)
#find_package(Boost COMPONENTS system thread coroutine context REQUIRED)
find_package(CURL REQUIRED)
find_package(jsoncpp REQUIRED)
#FIND_PACKAGE(Boost COMPONENTS python${PYTHON_VERSION_SUFFIX})

include_directories(includes
                    ${PCL_INCLUDE_DIRS}) #${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_library(websocket src/websocket.cpp)
#target_link_libraries(websocket ${PCL_LIBRARIES})


add_executable(${PROJECT_NAME} "src/main.cpp")
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${CURL_LIBRARIES} jsoncpp)

add_library(WebSocketClient "src/websocket.cpp")
target_link_libraries(WebSocketClient ${CURL_LIBRARIES} ${WEBSOCKETPP_LIBRARIES})
add_library(base64 "src/base64.cpp")
target_link_libraries(base64)

add_executable(testweb "src/test.cpp")
target_link_libraries(testweb WebSocketClient base64 jsoncpp  ${PCL_LIBRARIES}) #${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(testBase64 "src/base64Test.cpp")
target_link_libraries(testBase64 base64)

add_executable(websocketClTest "src/websockTest.cpp")
target_link_libraries(websocketClTest base64 ${BOOST_LIBRARIES} ${WEBSOCKETPP_LIBRARIES})
