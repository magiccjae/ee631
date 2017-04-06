cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( inverse_depth )
find_package( OpenCV REQUIRED )

add_executable(camera_calibrate camera_calibrate.cpp )
add_executable(inverse_depth inverse_depth.cpp )

target_link_libraries(camera_calibrate ${OpenCV_LIBS} )
target_link_libraries(inverse_depth ${OpenCV_LIBS} )
