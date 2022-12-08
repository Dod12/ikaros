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

// use the ikaros namespace to access the math library
// this is preferred to using <cmath>

using namespace ikaros;
using namespace ODrive;

void
ODriveController::Init()
{
    // Set parameters for motor controller
    Bind(serial_port, "serial_port");
    Bind(baud_rate, "baud_rate");
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

    // Setup serial communication
    odrive = new ODriveSerial(serial_port, baud_rate, 50);
    std::cout << "ODrive serial port: " << serial_port << std::endl;

    // Test connnection
    try 
    {
        float v = odrive->ReadFloat("r vbus_voltage");
        std::cout << "ODrive vbus voltage: " << v << std::endl;
    } catch (std::exception& e) {
        std::cout << "ODrive connection failed: " << e.what() << std::endl;
        return;
    }

    // Cast to enum to prevent overflow
    control_mode = (int) (ControlMode) control_mode;
    input_mode = (int) (InputMode) input_mode;

    // Set parameters for motor controller
    odrive->SetParameter("contoller.config.control_mode", control_mode);
    odrive->SetParameter("contoller.config.input_mode", input_mode);
    odrive->SetParameter("contoller.config.input_filter_bandwidth", input_filter_bandwidth);
    odrive->SetParameter("contoller.config.vel_ramp_rate", vel_ramp_rate);

    auto control_modes = odrive->GetParameterInt("contoller.config.control_mode");
    auto input_modes = odrive->GetParameterInt("contoller.config.input_mode");

    std::cout << "Control mode axis 0: " << control_modes.left << ", control mode axis 1: " << control_modes.right << std::endl;
    std::cout << "Input mode axis 0: " << control_modes.left << ", input mode axis 1: " << control_modes.right << std::endl;

    
    // Calibrate motors and set to closed loop control
    odrive->SetState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE, 10000, true);
    odrive->SetState(AXIS_STATE_CLOSED_LOOP_CONTROL, 2000, false);

    // Check axis state
    try 
    {
        auto axis_states = odrive->GetParameterInt("current_state");
        std::cout << "Axis 0 state: " << axis_states.left << ", axis 1 state: " << axis_states.right << std::endl;
        if (axis_states.left != AXIS_STATE_CLOSED_LOOP_CONTROL || axis_states.right != AXIS_STATE_CLOSED_LOOP_CONTROL) {
            std::cout << "Axis not in closed loop control" << std::endl;
            return;
        }
    } catch (std::exception& e) {
        std::cerr << "Failed to get axis state: " << e.what() << std::endl;
        return;
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
    static const float turns_per_meter = 1 / (wheel_circumference * gear_reduction);

    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL) 
    {
        // We need to invert the left desired pos, since the axes are mirrored
        fprintf(stderr, "Got command: %f, %f\r\n", -target_pos_array[0] * turns_per_meter, target_pos_array[1] * turns_per_meter);
        odrive->SetPosition({-target_pos_array[0] * turns_per_meter, target_pos_array[1] * turns_per_meter}, {speed, speed});
    } 
    else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL) 
    { 
        odrive->SetVelocity({-target_vel_array[0] * turns_per_meter, target_vel_array[1] * turns_per_meter});
    }
    else if (control_mode == ControlMode::CONTROL_MODE_TORQUE_CONTROL)
    {
        odrive->SetCurrent({-target_torque_array[0], target_torque_array[1]});
    }

    // Read position and velocity
    auto [left, right] = odrive->GetFeedback();

    pos_array[0] = -left.position / turns_per_meter;
    pos_array[1] = right.position / turns_per_meter;
    vel_array[0] = -left.velocity / turns_per_meter;
    vel_array[1] = right.velocity / turns_per_meter;
}

ODriveController::~ODriveController()
{
    delete odrive;
}

// Install the module. This code is executed during start-up.

static InitClass init("ODriveController", &ODriveController::Create, "Source/UserModules/EpiMove/ODriveController/");
