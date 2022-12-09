set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(SL-Lidar)

if (SL_LIDAR_LIB_FOUND)
	set(SOURCE
		${SOURCE}
		${CMAKE_CURRENT_SOURCE_DIR}/LidarSensor.cc
		PARENT_SCOPE
		)

	set(MODULES_INCLUDS
		${MODULES_INCLUDS}
		${SL_LIDAR_INCLUDE_DIR}
		PARENT_SCOPE
		)

	set(MODULES_LIBS
		${MODULES_LIBS}
		${SL_LIDAR_LIBRARIES}
		PARENT_SCOPE
		)

else()
	message(STATUS "Ikaros: SL-Lidar libs not found. LidarSensor module is not included into the build. Visit www.ikaros-project.org for information of module dependencies." )
endif()
