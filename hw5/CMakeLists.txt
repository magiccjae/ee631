cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw5 )
find_package( OpenCV REQUIRED )

add_executable(optical_flow optical_flow.cpp )

target_link_libraries(optical_flow ${OpenCV_LIBS} )
