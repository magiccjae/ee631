cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw3 )
find_package( OpenCV REQUIRED )

add_executable(intrinsic intrinsic.cpp )
add_executable(stereo_calibration stereo_calibration.cpp )
add_executable(epipolar_lines epipolar_lines.cpp )
add_executable(rectification rectification.cpp )

target_link_libraries(intrinsic ${OpenCV_LIBS} )
target_link_libraries(stereo_calibration ${OpenCV_LIBS} )
target_link_libraries(epipolar_lines ${OpenCV_LIBS} )
target_link_libraries(rectification ${OpenCV_LIBS} )
