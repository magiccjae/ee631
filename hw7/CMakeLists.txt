cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw7 )
find_package( OpenCV REQUIRED )

add_executable(no_info no_info.cpp )

target_link_libraries(no_info ${OpenCV_LIBS} )
