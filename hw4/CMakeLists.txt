cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw4 )
find_package( OpenCV REQUIRED )

add_executable(3d_measurement 3d_measurement.cpp )

target_link_libraries(3d_measurement ${OpenCV_LIBS} )
