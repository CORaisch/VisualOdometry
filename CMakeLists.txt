cmake_minimum_required (VERSION 3.5)
project (Visual_Odometry)

set (CMAKE_CXX_STANDARD 11)

find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS} include/)

file(GLOB src_files src/*.cpp)
file(GLOB hdr_file include/*.hpp include/*.h)

add_executable (visodom ${src_files} ${hdr_files})
target_link_libraries (visodom ${OpenCV_LIBS} pthread)