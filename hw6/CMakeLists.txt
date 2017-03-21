cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)
project( hw6 )
find_package( OpenCV REQUIRED )

add_executable(stm_no_params stm_no_params.cpp )
add_executable(stm_intrinsic stm_intrinsic.cpp )

target_link_libraries(stm_no_params ${OpenCV_LIBS} )
target_link_libraries(stm_intrinsic ${OpenCV_LIBS} )
