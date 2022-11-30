set(CMAKE_INCLUDE_CURRENT_DIR ON)

#find_package(libusb)

if (true)

	set(SOURCE
		${SOURCE}
		${CMAKE_CURRENT_SOURCE_DIR}/odrive
		${CMAKE_CURRENT_SOURCE_DIR}/ODriveController.cc
		${CMAKE_CURRENT_SOURCE_DIR}/odrive/odrive.cpp
		PARENT_SCOPE
	)

	set(MODULES_INCLUDS
		${MODULES_INCLUDS}
		#${LIBUSB_INCLUDE_DIR}
		/opt/homebrew/Cellar/libusb/1.0.26/include/libusb-1.0/
		PARENT_SCOPE
		)

	set(MODULES_LIBS
		${MODULES_LIBS}
		#${LIBUSB_LIBRARIES}
		/opt/homebrew/Cellar/libusb/1.0.26/lib/libusb-1.0.0.dylib
		PARENT_SCOPE
		)

else()
	message(STATUS "Ikaros: Libusb libs not found. ODriveController module is not included into the build. Visit www.ikaros-project.org for information of module dependencies." )
endif()