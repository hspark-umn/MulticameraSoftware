cmake_minimum_required(VERSION 2.8)
project( sift++ )
set(CMAKE_BUILD_TYPE Release)
#set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
# include_directories (${OpenCV_INCLUDE_DIRS} )
#include_directories ( /panfs/roc/itascasoft/opencv/3.0.0/include/opencv )
#include_directories ( /panfs/roc/itascasoft/opencv/3.0.0/include/opencv2 )
#INCLUDE ( /usr/share/cmake/Modules/FindPkgConfig.cmake )
find_package( OpenCV REQUIRED )
include_directories (${OpenCV_INCLUDE_DIRS} )
add_executable( sift++ SIFT++.cpp sift.cpp )
target_link_libraries( sift++
	${OpenCV_LIBS}
)
