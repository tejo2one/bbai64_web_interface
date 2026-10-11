cmake_minimum_required(VERSION 3.10)
project(web_teleop_server CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread")

find_package(OpenCV REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(web_teleop_server src/web_teleop_server.cpp)
target_link_libraries(web_teleop_server ${OpenCV_LIBS} pthread)
