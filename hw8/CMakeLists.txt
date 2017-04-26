cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw8 )
find_package( OpenCV REQUIRED )

add_executable(visual_odometry visual_odometry.cpp )
add_executable(vo_task2 vo_task2.cpp )

target_link_libraries(visual_odometry ${OpenCV_LIBS} )
target_link_libraries(vo_task2 ${OpenCV_LIBS} )
