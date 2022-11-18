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
    Bind(circumference, "wheel_circumference");

    control_mode = (ControlMode) control_mode;
    input_mode = (InputMode) input_mode;

    io(target_array, target_array_size, "TARGET_ARRAY");
    set_array(target_array, 0, target_array_size);

    io(pos_array, pos_array_size, "POS_ESTIM");
    io(vel_array, vel_array_size, "VEL_ESTIM");

    io(robot_speed, robot_speed_size, "ROBOT_SPEED");

    offset_array_size = 2;
    offset_array = create_array(offset_array_size);

    // Init driver
    odrive = odrive::ODrive();
    if (odrive.search_device() != odrive::ReturnStatus::STATUS_SUCCESS) {
        fprintf(stderr, "Failed to find ODrive\r\n");
        return;
    }

    for (auto offset : std::vector<int>{0, per_axis_offset})

    {
        // Set desired mode (position/veolcity) and input filtering
        odrive.write(AXIS__CONTROLLER__CONFIG__CONTROL_MODE + offset, control_mode);

        odrive.write(AXIS__CONTROLLER__CONFIG__INPUT_MODE + offset, input_mode);

        // Set params for filter if desired and compatible with control mode
        if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL && input_mode == InputMode::INPUT_MODE_POS_FILTER) {
            odrive.write(AXIS__CONTROLLER__CONFIG__INPUT_FILTER_BANDWIDTH + offset, input_filter_bandwidth);

        } else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL && input_mode == InputMode::INPUT_MODE_VEL_RAMP) {
            odrive.write(AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE + offset, vel_ramp_rate);

        } else {
            fprintf(stderr, "Control and input modes are not compatible, got control mode %i and input mode %i", control_mode, input_mode);
        }
    }

    // Disable closed loop controls for reseting the encoder index
    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL) {
        for (int i = 0; i < offset_array_size; ++i) {
            odrive.read(AXIS__ENCODER__POS_ESTIMATE + i*per_axis_offset, offset_array[i]);
            odrive.read(AXIS__CONTROLLER__)
        }
    } else {
        set_array(offset_array, 0, offset_array_size);
    }
}

void
ODriveController::Tick()
{   
    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL) {
        // We need to invert the left desired pos, since the axes are mirrored
        float target_left = circumference * (-1 * target_array[0]) + offset_array[0];
        float target_right = circumference * target_array[1] + offset_array[1];
        odrive.write(AXIS__CONTROLLER__INPUT_POS, target_left);
        odrive.write(AXIS__CONTROLLER__INPUT_POS + per_axis_offset, target_right);
    } else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL) { 
        odrive.write(AXIS__CONTROLLER__INPUT_VEL, target_array[0]);
        odrive.write(AXIS__CONTROLLER__INPUT_VEL + per_axis_offset, target_array[1]);
    } else if (control_mode == ControlMode::CONTROL_MODE_TORQUE_CONTROL) {
        odrive.write(AXIS__CONTROLLER__INPUT_TORQUE, target_array[0]);
        odrive.write(AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset, target_array[1]);
    }

    float left_vel, right_vel;
    odrive.read(AXIS__ENCODER__VEL_ESTIMATE, left_vel);
    odrive.read(AXIS__ENCODER__VEL_ESTIMATE + per_axis_offset, right_vel);
    vel_array[0] = -1 * left_vel / circumference;
    vel_array[1] = right_vel / circumference;
    float left_pos, right_pos;
    odrive.read(AXIS__ENCODER__POS_ESTIMATE, left_pos);
    odrive.read(AXIS__ENCODER__POS_ESTIMATE + per_axis_offset, right_pos);
    pos_array[0] = circumference * (-1 * left_pos) - offset_array[0]; 
    pos_array[1] = circumference * right_pos - offset_array[1];
}

ODriveController::~ODriveController()
{
    // ODrive driver handles relasing the USB context
    destroy_array(offset_array);
}

// Install the module. This code is executed during start-up.

static InitClass init("ODriveController", &ODriveController::Create, "Source/UserModules/ODriveController/");
