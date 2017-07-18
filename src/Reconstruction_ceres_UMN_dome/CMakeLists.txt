cmake_minimum_required(VERSION 2.8)
project( Reconstruction_ceres_UMN_Dome )
FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
INCLUDE ( /usr/share/cmake-3.5/Modules/FindPkgConfig.cmake )
find_package( OpenCV REQUIRED )
pkg_check_modules( REQUIRED flann )
find_package( OpenMP REQUIRED)
find_package( Boost REQUIRED COMPONENTS filesystem )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
add_executable( Reconstruction_ceres_UMN_Dome Reconstruction3D_SIGGRAPH2012_robolounge.cpp
		CeresUtility.cpp
		DataUtility.cpp
		epnp.cpp
		MathUtility.cpp
		)
target_link_libraries( Reconstruction_ceres_UMN_Dome -lboost_system
	${OpenCV_LIBS}
	${Boost_FILESYSTEM_LIBRARY}
	${CERES_LIBRARIES}
)
