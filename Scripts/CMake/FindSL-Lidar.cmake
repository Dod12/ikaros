# www.ikaros-project.org
#
# Example how to write a cmake module to use a external library if the library is not yet included in the official cmake script.
# To check included modules use "cmake --help-module-list"
#
#
# This module defines:
# SL_LIDAR_INCLUDE_DIR
# SL_LIDAR_LIBRARIES
# SL_LIDAR_LIB_FOUND


# Specific parameters for different OS
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
# Find one header file and lib
#############################################

# Find header files
find_path(SL_LIDAR_INCLUDE_DIR
        NAMES
        rplidar.h
        PATHS
        /usr/local/include
        /usr/include
        PATH_SUFFIXES
        sl_lidar_sdk
)

# Find lib file
find_library(SL_LIDAR_LIBRARIES
        NAMES
        libsl_lidar_sdk.a
        PATHS
        /usr/local/lib
        /usr/lib
)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
# Find one header file and lib 
#############################################
# Tested in a raspberry

# Find header files
find_path(SL_LIDAR_INCLUDE_DIR
        NAMES
        rplidar.h
        PATHS
        /usr/local/include
        /usr/include
        PATH_SUFFIXES
        sl_lidar_sdk
)

# Find lib file
find_library(SL_LIDAR_LIBRARIES
        NAMES
        libsl_lidar_sdk.a
        PATHS
        /usr/local/lib
        /usr/lib
        PATH_SUFFIXES
        sl_lidar_sdk
)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
endif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")

if (SL_LIDAR_INCLUDE_DIR AND SL_LIDAR_LIBRARIES)
  message(STATUS "Found Sl-Lidar SDK: Includes: ${SL_LIDAR_INCLUDE_DIR} Libraries: ${SL_LIDAR_LIBRARIES}")
  set(SL_LIDAR_LIB_FOUND "YES" )
endif ()
#############################################

