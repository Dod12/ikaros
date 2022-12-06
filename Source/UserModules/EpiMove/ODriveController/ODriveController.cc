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

template <typename ValueType>
void safe_write(odrive::ODrive& odrive, const short endpoint, const ValueType value) {
    if (odrive.write(endpoint, value) == odrive::ReturnStatus::STATUS_ERROR) {
        std::cerr << "Error writing " << value << " to endpoint " << endpoint << std::endl;
    }
}

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

    std::cout << "Input mode: " << input_mode << ", control mode: " << control_mode << std::endl;
    std::cout << "Wheel circumference: " << wheel_circumference << ", gear reduction: " << gear_reduction << std::endl;
    std::cout << "Max speed: " << speed << std::endl;

    // Cast to enum to prevent overflow
    control_mode = (int) (ControlMode) control_mode;
    input_mode = (int) (InputMode) input_mode;

    // Set paramters for each axis
    for (int axis_offset : {0, (int) per_axis_offset}) {
        /*
        safe_write(odrive, AXIS__MOTOR__CONFIG__CURRENT_LIM + axis_offset, 10);
        safe_write(odrive, AXIS__CONTROLLER__CONFIG__VEL_LIMIT + axis_offset, 10);
        safe_write(odrive, AXIS__MOTOR__CONFIG__CALIBRATION_CURRENT + axis_offset, 10);
        safe_write(odrive, AXIS__MOTOR__CONFIG__POLE_PAIRS + axis_offset, 7);
        safe_write(odrive, AXIS__MOTOR__CONFIG__TORQUE_CONSTANT + axis_offset, (float) 8.27 / 150);
        safe_write(odrive, AXIS__MOTOR__CONFIG__MOTOR_TYPE + axis_offset, (int) MOTOR_TYPE_HIGH_CURRENT);
        safe_write(odrive, AXIS__ENCODER__CONFIG__CPR + axis_offset, 8192);
        */
        safe_write(odrive, AXIS__CONTROLLER__CONFIG__CONTROL_MODE + axis_offset, control_mode);
        safe_write(odrive, AXIS__CONTROLLER__CONFIG__INPUT_MODE + axis_offset, input_mode);
        safe_write(odrive, AXIS__CONTROLLER__CONFIG__INPUT_FILTER_BANDWIDTH + axis_offset, input_filter_bandwidth);
        safe_write(odrive, AXIS__CONTROLLER__CONFIG__VEL_RAMP_RATE + axis_offset, vel_ramp_rate);
    }

    float control_mode_axis0, control_mode_axis1;
    float input_mode_axis0, input_mode_axis1;

    odrive.read(AXIS__CONTROLLER__CONFIG__CONTROL_MODE, control_mode_axis0);
    odrive.read(AXIS__CONTROLLER__CONFIG__CONTROL_MODE + per_axis_offset, control_mode_axis1);
    odrive.read(AXIS__CONTROLLER__CONFIG__INPUT_MODE, input_mode_axis0);
    odrive.read(AXIS__CONTROLLER__CONFIG__INPUT_MODE + per_axis_offset, input_mode_axis1);

    std::cout << "Control mode axis 0: " << control_mode_axis0 << ", control mode axis 1: " << control_mode_axis1 << std::endl;
    std::cout << "Input mode axis 0: " << input_mode_axis0 << ", input mode axis 1: " << input_mode_axis1 << std::endl;

    /*
    // Calibrate motors
    safe_write(odrive, AXIS__REQUESTED_STATE, (int) AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
    safe_write(odrive, AXIS__REQUESTED_STATE + per_axis_offset, (int) AXIS_STATE_FULL_CALIBRATION_SEQUENCE);

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // Set closed loop control
    safe_write(odrive, AXIS__REQUESTED_STATE, (int) AXIS_STATE_CLOSED_LOOP_CONTROL);
    safe_write(odrive, AXIS__REQUESTED_STATE + per_axis_offset, (int) AXIS_STATE_CLOSED_LOOP_CONTROL);

    int axis0_state, axis1_state;
    do {
        std::cerr << "Waiting for ODrive to be ready" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        odrive.read(AXIS__CURRENT_STATE, axis0_state);
        odrive.read(AXIS__CURRENT_STATE + per_axis_offset, axis1_state);
    } while (axis0_state != AXIS_STATE_CLOSED_LOOP_CONTROL || axis1_state != AXIS_STATE_CLOSED_LOOP_CONTROL);
    */

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
    static const float turns_per_meter = 1 / (wheel_circumference * gear_reduction);

    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL) {
        // We need to invert the left desired pos, since the axes are mirrored
        fprintf(stderr, "Got command: %f, %f\r\n", -target_pos_array[0] * turns_per_meter, target_pos_array[1] * turns_per_meter);
        safe_write(odrive, AXIS__CONTROLLER__INPUT_POS + 0, (float) -target_pos_array[0] * turns_per_meter);
        safe_write(odrive, AXIS__CONTROLLER__INPUT_POS + per_axis_offset, (float) target_pos_array[1] * turns_per_meter);
    } else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL) { 
        safe_write(odrive, AXIS__CONTROLLER__INPUT_VEL + 0, (float) -target_vel_array[0] * turns_per_meter);
        safe_write(odrive, AXIS__CONTROLLER__INPUT_VEL + per_axis_offset, (float) target_vel_array[1] * turns_per_meter);
    } else if (control_mode == ControlMode::CONTROL_MODE_TORQUE_CONTROL) {
        safe_write(odrive, AXIS__CONTROLLER__INPUT_TORQUE + 0, (float) -target_torque_array[0]);
        safe_write(odrive, AXIS__CONTROLLER__INPUT_TORQUE + per_axis_offset, (float) target_torque_array[1]);
    }

    // Read position and velocity
    float left_pos, right_pos;
    float left_vel, right_vel;
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

static InitClass init("ODriveController", &ODriveController::Create, "Source/UserModules/EpiMove/ODriveController/");
