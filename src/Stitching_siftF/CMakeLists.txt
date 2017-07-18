cmake_minimum_required(VERSION 2.8)
project( Stitching_sift )
INCLUDE (/usr/share/cmake-3.5/Modules/FindPkgConfig.cmake )
find_package( OpenCV REQUIRED )
pkg_check_modules( REQUIRED flann )
find_package( OpenMP REQUIRED)
find_package( Boost REQUIRED COMPONENTS filesystem )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
add_executable( Stitching_sift SIFT_LOWES_Fisheye_FLANN.cpp 
			 MathUtility.cpp
			 DataUtility.cpp )
target_link_libraries( Stitching_sift -lboost_system 
	${OpenCV_LIBS}
	${Boost_FILESYSTEM_LIBRARY}
)
