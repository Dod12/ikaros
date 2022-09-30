//
//	ODriveController.cc		This file is a part of the IKAROS project
//
//    Copyright (C) 2022 Daniel Calström Schad
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//
//    See http://www.ikaros-project.org/ for more information.
//

#include "ODriveController.h"
#include <iostream>

// use the ikaros namespace to access the math library
// this is preferred to using <cmath>

using namespace ikaros;
using namespace odrive::endpoints;
using namespace odrive::enums;

void
ODriveController::Init()
{
    // To get the parameters from the IKC file, use the Bind
    // function for each parameter. The parameters are initialized
    // from the IKC and can optionally be changed from the
    // user interface while Ikaros is running. If the parameter is not
    // set, the default value specified in the ikc-file will be used instead.

    Bind(input_mode, "input_mode");
    Bind(control_mode, "control_mode");
    Bind(input_filter_bandwidth, "input_filter_bandwidth");
    Bind(vel_ramp_rate, "vel_ramp_rate");

    control_mode = (ControlMode) control_mode;
    input_mode = (InputMode) input_mode;

    //io(target_array, target_array_size, "TARGET_ARRAY");
    target_array = create_array(2);
    target_array_size = 2;
    set_array(target_array, 0, target_array_size);

    io(odom_array, odom_array_size, "ODOM_ARRAY");

    odrive = odrive::ODrive();
    odrive.search_device();

    odrive.write(AXIS__CONTROLLER__CONFIG__CONTROL_MODE, control_mode);
    odrive.write(AXIS__CONTROLLER__CONFIG__INPUT_MODE, input_mode);

    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL && 
        input_mode == InputMode::INPUT_MODE_POS_FILTER) {
        odrive.write(AXIS__CONTROLLER__CONFIG__INPUT_FILTER_BANDWIDTH, input_filter_bandwidth);
        odrive.write(AXIS__CONTROLLER__CONFIG__INPUT_FILTER_BANDWIDTH + per_axis_offset, input_filter_bandwidth);
    } else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL &&
               input_mode == InputMode::INPUT_MODE_VEL_RAMP) {
        odrive.write(AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE, vel_ramp_rate);
        odrive.write(AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE + per_axis_offset, vel_ramp_rate);
    }
}

void
ODriveController::Tick()
{           
    float target_left = target_array[0];
    float target_right = target_array[1];
    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL) {
        odrive.write(AXIS__CONTROLLER__INPUT_POS, target_left);
        odrive.write(AXIS__CONTROLLER__INPUT_POS + per_axis_offset, target_right);
    } else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL) {
        odrive.write(AXIS__CONTROLLER__INPUT_VEL, target_left);
        odrive.write(AXIS__CONTROLLER__INPUT_VEL + per_axis_offset, target_right);
    } else if (control_mode == ControlMode::CONTROL_MODE_TORQUE_CONTROL) {
        odrive.write(AXIS__CONTROLLER__INPUT_TORQUE, target_left);
        odrive.write(AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset, target_right);
    }

    odrive.read(AXIS__ENCODER__POS_ESTIMATE, odom_array[0]);
    odrive.read(AXIS__ENCODER__POS_ESTIMATE + per_axis_offset, odom_array[1]);
}

ODriveController::~ODriveController()
{
    // ODrive driver handles relasing the USB context
    destroy_array(target_array);
}

void ODriveController::Command(std::string s, float x, float y, std::string value) {
    std::cout << "Got command: " << s << std::endl;
    if (s == "left") left();
    else if (s == "right") right();
    else if (s == "forward") forward();
    else if (s == "back") back();
}

// The two motors are placed in a mirrored fashion in the robot platform, so that the left side is inverted.

void ODriveController::left() {
    target_array[0] += 1;
    target_array[1] += 1;
    float target_left = target_array[0];
    float target_right = target_array[1];
    odrive.write(AXIS__CONTROLLER__INPUT_POS, target_left);
    odrive.write(AXIS__CONTROLLER__INPUT_POS + per_axis_offset, target_right);
}

void ODriveController::right() {
    target_array[0] += -1;
    target_array[1] += -1;
    float target_left = target_array[0];
    float target_right = target_array[1];
    odrive.write(AXIS__CONTROLLER__INPUT_POS, target_left);
    odrive.write(AXIS__CONTROLLER__INPUT_POS + per_axis_offset, target_right);
}

void ODriveController::back() {
    target_array[0] += 1;
    target_array[1] += -1;
    float target_left = target_array[0];
    float target_right = target_array[1];
    odrive.write(AXIS__CONTROLLER__INPUT_POS, target_left);
    odrive.write(AXIS__CONTROLLER__INPUT_POS + per_axis_offset, target_right);
}   

void ODriveController::forward() {
    target_array[0] += -1;
    target_array[1] += 1;
    float target_left = target_array[0];
    float target_right = target_array[1];
    odrive.write(AXIS__CONTROLLER__INPUT_POS, target_left);
    odrive.write(AXIS__CONTROLLER__INPUT_POS + per_axis_offset, target_right);
}

// Install the module. This code is executed during start-up.

static InitClass init("ODriveController", &ODriveController::Create, "Source/UserModules/ODriveController/");
