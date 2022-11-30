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

    // Init driver
    odrive = odrive::ODrive();
    if (odrive.search_device() != odrive::ReturnStatus::STATUS_SUCCESS) {
        fprintf(stderr, "Failed to find ODrive\r\n");
        return;
    }

    // Set parameters for motor controller
    Bind(input_mode, "input_mode");
    Bind(control_mode, "control_mode");
    Bind(input_filter_bandwidth, "input_filter_bandwidth");
    Bind(vel_ramp_rate, "vel_ramp_rate");
    Bind(wheel_circumference, "wheel_circumference");
    Bind(gear_reduction, "gear_reduction");
    Bind(speed, "start_speed");

    // Cast to enum to prevent overflow
    control_mode = (ControlMode) control_mode;
    input_mode = (InputMode) input_mode;

    // Set paramters for each axis
    for (auto axis_offset : {0, 1}) {
        odrive.write(AXIS__MOTOR__CONFIG__CURRENT_LIM + axis_offset, 5);
        odrive.write(AXIS__CONTROLLER__CONFIG__VEL_LIMIT + axis_offset, 10);
        odrive.write(AXIS__MOTOR__CONFIG__CALIBRATION_CURRENT + axis_offset, 5);
        odrive.write(AXIS__MOTOR__CONFIG__POLE_PAIRS + axis_offset, 7);
        odrive.write(AXIS__MOTOR__CONFIG__TORQUE_CONSTANT + axis_offset, (float) 8.27 / 150);
        odrive.write(AXIS__MOTOR__CONFIG__MOTOR_TYPE + axis_offset, (int) MOTOR_TYPE_HIGH_CURRENT);
        odrive.write(AXIS__ENCODER__CONFIG__CPR + axis_offset, 8192);
        odrive.write(AXIS__CONTROLLER__CONFIG__CONTROL_MODE + axis_offset, control_mode);
        odrive.write(AXIS__CONTROLLER__CONFIG__INPUT_MODE + axis_offset, input_mode);
        odrive.write(AXIS__CONTROLLER__CONFIG__INPUT_FILTER_BANDWIDTH + axis_offset, input_filter_bandwidth);
        odrive.write(AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE + axis_offset, vel_ramp_rate);
        odrive.write(AXIS__REQUESTED_STATE + axis_offset, (int) AXIS_STATE_IDLE);
        odrive.write(AXIS__REQUESTED_STATE + axis_offset, (int) AXIS_STATE_MOTOR_CALIBRATION);
        odrive.write(AXIS__REQUESTED_STATE + axis_offset, (int) AXIS_STATE_ENCODER_INDEX_SEARCH);
        odrive.write(AXIS__REQUESTED_STATE + axis_offset, (int) AXIS_STATE_ENCODER_OFFSET_CALIBRATION);
        odrive.write(AXIS__REQUESTED_STATE + axis_offset, (int) AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    // Bind input and output arrays
    io(target_pos_array, target_pos_array_size, "POS_TARGET"); // Target position in m on each axis, 0=left, 1=right
    io(target_vel_array, target_vel_array_size, "VEL_TARGET"); // Target velocity in m/s on each axis, 0=left, 1=right
    io(target_torque_array, target_torque_array_size, "TORQUE_TARGET"); // Target torque on each axis, 0=left, 1=right

    io(pos_array, pos_array_size, "POS_ESTIM"); 
    io(vel_array, vel_array_size, "VEL_ESTIM"); 
}

void
ODriveController::Tick()
{   
    static const int turns_per_meter = 1 / (wheel_circumference * gear_reduction);

    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL) {
        // We need to invert the left desired pos, since the axes are mirrored
        odrive.write(AXIS__CONTROLLER__POS_SETPOINT + 0, -target_pos_array[0] * turns_per_meter);
        odrive.write(AXIS__CONTROLLER__POS_SETPOINT + per_axis_offset, target_pos_array[1] * turns_per_meter);
    } else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL) { 
        odrive.write(AXIS__CONTROLLER__VEL_SETPOINT + 0, -target_vel_array[0] * turns_per_meter);
        odrive.write(AXIS__CONTROLLER__VEL_SETPOINT + per_axis_offset, target_vel_array[1] * turns_per_meter);
    } else if (control_mode == ControlMode::CONTROL_MODE_TORQUE_CONTROL) {
        odrive.write(AXIS__CONTROLLER__TORQUE_SETPOINT + 0, -target_torque_array[0]);
        odrive.write(AXIS__CONTROLLER__TORQUE_SETPOINT + per_axis_offset, target_torque_array[1]);
    }

    // Read position and velocity
    int left_pos, right_pos;
    int left_vel, right_vel;
    odrive.read(AXIS__ENCODER__POS_ESTIMATE + 0, left_pos);
    odrive.read(AXIS__ENCODER__POS_ESTIMATE + per_axis_offset, right_pos);
    odrive.read(AXIS__ENCODER__VEL_ESTIMATE + 0, left_vel);
    odrive.read(AXIS__ENCODER__VEL_ESTIMATE + per_axis_offset, right_vel);

    pos_array[0] = -left_pos / turns_per_meter;
    pos_array[1] = right_pos / turns_per_meter;
    vel_array[0] = -left_vel / turns_per_meter;
    vel_array[1] = right_vel / turns_per_meter;
}

ODriveController::~ODriveController()
{
    // ODrive driver handles relasing the USB context
}

// Install the module. This code is executed during start-up.

static InitClass init("ODriveController", &ODriveController::Create, "Source/UserModules/ODriveController/");
