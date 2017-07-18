cmake_minimum_required(VERSION 2.4.6)
set(CMAKE_CXX_STANDARD 11)

# arvacquisitiontest_jpg.cpp) arvcameratest_jpg.cpp
#  arvexample_jpg.c arvheartbeattest_jpg.cpp
set(targetname "arv_stream")

file(GLOB SRCS "*.cpp" "*.h")

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMakeModules )
FIND_PACKAGE(Aravis)
find_package(OpenCV)
find_package(Threads)
#find_package(LibSSH)

  find_path(LIBSSH_INCLUDE_DIR
    NAMES
      libssh/libssh.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
      ${CMAKE_INCLUDE_PATH}
      ${CMAKE_INSTALL_PREFIX}/include
    )

  find_library(SSH_LIBRARY
    NAMES
      ssh
      libssh
    PATHS
      /usr/lib
      /usr/local/lib
      /opt/local/lib
      /sw/lib
      ${CMAKE_LIBRARY_PATH}
      ${CMAKE_INSTALL_PREFIX}/lib
  )

  if (LIBSSH_INCLUDE_DIR AND SSH_LIBRARY)
    set(SSH_FOUND TRUE)
  endif (LIBSSH_INCLUDE_DIR AND SSH_LIBRARY)

  set(LIBSSH_INCLUDE_DIRS
    ${LIBSSH_INCLUDE_DIR}
  )

  if (SSH_FOUND)
    set(LIBSSH_LIBRARIES
      ${LIBSSH_LIBRARIES}
      ${SSH_LIBRARY}
    )
  endif (SSH_FOUND)

#find_package(DBus REQUIRED)

#include(FindPkgConfig)
find_package(PkgConfig)
pkg_check_modules(GLIB glib-2.0)
pkg_check_modules(DBus dbus-1)

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4
                               gstreamer-sdp-1.0>=1.4
                               gstreamer-video-1.0>=1.4
                               gstreamer-app-1.0>=1.4)



include_directories(${ARAVIS_INCLUDE_PATH} ${GLIB_INCLUDE_DIRS} ${OPENCV_INCLUDE_DIRS} ${DBUS_INCLUDE_DIRS} ${GST_INCLUDE_DIRS} ${LIBSSH_INCLUDE_DIR})
#link_directories (${GLIB_LIBRARIES})
add_executable (${targetname} ${SRCS} )
#  sftp.h com.h sftp.cpp arv_main.cpp
# arvacquisitiontest_jpg.cpp) arvcameratest_jpg.cpp
#  arvexample.c

add_definitions (${GLIB_CFLAGS_OTHER} ${GST_CFLAGS})
target_link_libraries(${targetname} ${ARAVIS_LIBRARY} ${GLIB_LIBRARIES} ${OpenCV_LIBS} ${DBUS_LIBRARIES} ${GST_LIBRARIES} ${LIBSSH_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
#link_libraries(${ARAVIS_LIBRARY} ${GLIB_LIBRARIES} ${OpenCV_LIBS} ${DBUS_LIBRARIES})

