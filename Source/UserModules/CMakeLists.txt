set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_subdirectory(MinimalModule)
add_subdirectory(MyModule)
add_subdirectory(MotorControl)
add_subdirectory(CameraSimulate)
add_subdirectory(MTPupilDemo)

# New code
add_subdirectory(EpiMove)

# Old code
#add_subdirectory(ODriveController)
add_subdirectory(PathIntegrator)
#add_subdirectory(TargetArrayControls)
add_subdirectory(OccupancyMap)
add_subdirectory(LidarSensor)


set(SOURCE ${SOURCE} PARENT_SCOPE)
set(MODULES_INCLUDS ${MODULES_INCLUDS} PARENT_SCOPE)
set(MODULES_LIBS ${MODULES_LIBS} PARENT_SCOPE)
