set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(ODriveController)
add_subdirectory(TargetArrayControls)
add_subdirectory(PathIntegrator)
add_subdirectory(LidarSensor)
add_subdirectory(OccupancyMap)
add_subdirectory(TransitionMatrix)


set(SOURCE ${SOURCE} PARENT_SCOPE)
set(MODULES_INCLUDS ${MODULES_INCLUDS} PARENT_SCOPE)
set(MODULES_LIBS ${MODULES_LIBS} PARENT_SCOPE)