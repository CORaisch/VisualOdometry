cmake_minimum_required (VERSION 3.5)
project (Visual_Odometry)

set (CMAKE_CXX_STANDARD 11)

find_package (OpenCV REQUIRED)
include_directories (${OpenCV_INCLUDE_DIRS} include/)

# collect all cpp and headers
file(GLOB src_files src/*.cpp)
file(GLOB hdr_file include/*.hpp include/*.h)

# remove main files from src_files list
list(REMOVE_ITEM src_files "${PROJECT_SOURCE_DIR}/src/main_kitti.cpp")
list(REMOVE_ITEM src_files "${PROJECT_SOURCE_DIR}/src/main_euroc.cpp")

# build and link visodom_kitti
add_executable (visodom_kitti src/main_kitti.cpp ${src_files} ${hdr_files})
target_link_libraries (visodom_kitti ${OpenCV_LIBS} pthread)

# build and link visodom_euroc
add_executable (visodom_euroc src/main_euroc.cpp ${src_files} ${hdr_files})
target_link_libraries (visodom_euroc ${OpenCV_LIBS} pthread)
