cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw4 )
find_package( OpenCV REQUIRED )

add_executable(3d_measurement 3d_measurement.cpp )
add_executable(ball_tracking ball_tracking.cpp )

target_link_libraries(3d_measurement ${OpenCV_LIBS} )
target_link_libraries(ball_tracking ${OpenCV_LIBS} )
