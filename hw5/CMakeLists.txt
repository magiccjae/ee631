cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw5 )
find_package( OpenCV REQUIRED )

add_executable(optical_flow optical_flow.cpp )
add_executable(feature_matching feature_matching.cpp )
add_executable(multi-frame_feature_tracking multi-frame_feature_tracking.cpp )

target_link_libraries(optical_flow ${OpenCV_LIBS} )
target_link_libraries(feature_matching ${OpenCV_LIBS} )
target_link_libraries(multi-frame_feature_tracking ${OpenCV_LIBS} )
