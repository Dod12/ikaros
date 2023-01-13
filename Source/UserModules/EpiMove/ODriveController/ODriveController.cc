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

    Notify(msg_debug, "Input mode: %d, control mode: %d", input_mode, control_mode);
    Notify(msg_debug, "Input filter bandwidth: %f, vel ramp rate: %f", input_filter_bandwidth, vel_ramp_rate);
    Notify(msg_debug, "Wheel circumference: %f, gear reduction: %f", wheel_circumference, gear_reduction);
    Notify(msg_debug, "Max speed: %f", speed);

    turns_per_meter = gear_reduction / wheel_circumference;
    meters_per_turn = 1 / turns_per_meter;

    // Setup serial communication
    odrive = new ODriveSerial(serial_port, baud_rate, 50);
    Notify(msg_debug, "Serial port: %s, baud rate: %d", serial_port.c_str(), baud_rate);

    // Reboot to make sure there are no problems
    odrive->ClearErrors();
    odrive->Reboot();
    delete odrive;
    std::this_thread::sleep_for(std::chrono::seconds(2));
    odrive = new ODriveSerial(serial_port, baud_rate, 50);

    // Test connnection
    try 
    {
        float v = odrive->ReadFloat("r vbus_voltage");
        Notify(msg_debug, "ODrive connection successful, vbus voltage: %f", v);
    } catch (std::exception& e) {
        Notify(msg_exception, "ODrive connection failed: %s", e.what());
    }

    // Cast to enum to prevent overflow
    control_mode = (int) (ControlMode) control_mode;
    input_mode = (int) (InputMode) input_mode;

    // Set parameters for motor controller
    odrive->SetParameter("controller.config.control_mode", control_mode);
    odrive->SetParameter("controller.config.input_mode", input_mode);
    odrive->SetParameter("controller.config.input_filter_bandwidth", input_filter_bandwidth);
    odrive->SetParameter("controller.config.vel_ramp_rate", vel_ramp_rate);

    auto control_modes = odrive->GetParameterInt("controller.config.control_mode");
    auto input_modes = odrive->GetParameterInt("controller.config.input_mode");
    
    // Calibrate motors and set to closed loop control
    if (odrive->Calibrate() != ReturnStatus::OK) {
        Notify(msg_fatal_error, "Failed to calibrate motors");
    }
    if (odrive->SetState(AXIS_STATE_CLOSED_LOOP_CONTROL) != ReturnStatus::OK) {
        Notify(msg_fatal_error, "Failed to set motors to closed loop control");
    }
    /*
    if (odrive->SetState(AXIS_STATE_FULL_CALIBRATION_SEQUENCE, 10000, true) != ReturnStatus::OK) {
        Notify(msg_fatal_error, "Failed to calibrate motors");
    }
    if (odrive->SetState(AXIS_STATE_CLOSED_LOOP_CONTROL, 2000, false) != ReturnStatus::OK) {
        Notify(msg_fatal_error, "Failed to set motors to closed loop control");
    }

    std::this_thread::sleep_for(std::chrono::seconds(2));
    */

    // Check axis state
    try 
    {
        auto axis_states = odrive->GetParameterInt("current_state");
        Notify(msg_debug, "Axis 0 state: %d, axis 1 state: %d", axis_states.left, axis_states.right);
        if (axis_states.left != AXIS_STATE_CLOSED_LOOP_CONTROL) {
            Notify(msg_warning, "Left axis not in closed loop control");
        }
        if (axis_states.right != AXIS_STATE_CLOSED_LOOP_CONTROL) {
            Notify(msg_warning, "Right axis not in closed loop control");
        }
    } catch (std::exception& e) {
        Notify(msg_fatal_error, "Failed to get axis state");
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

    if (control_mode == ControlMode::CONTROL_MODE_POSITION_CONTROL) 
    {
        // We need to invert the left desired pos, since the axes are mirrored
        Notify(msg_debug, "Setting position to %f, %f", -target_pos_array[0] * turns_per_meter, target_pos_array[1] * turns_per_meter);
        odrive->SetPosition({-target_pos_array[0] * turns_per_meter, target_pos_array[1] * turns_per_meter}); // TODO: Set speed based on confidence in the trajectory and the distance to the target
    } 
    else if (control_mode == ControlMode::CONTROL_MODE_VELOCITY_CONTROL) 
    {
        Notify(msg_debug, "Setting velocity to %f, %f", -target_vel_array[0] * turns_per_meter, target_vel_array[1] * turns_per_meter);
        odrive->SetVelocity({-target_vel_array[0] * turns_per_meter, target_vel_array[1] * turns_per_meter});
    }
    else if (control_mode == ControlMode::CONTROL_MODE_TORQUE_CONTROL)
    {
        Notify(msg_debug, "Setting torque to %f, %f", -target_torque_array[0], target_torque_array[1]);
        odrive->SetCurrent({-target_torque_array[0], target_torque_array[1]});
    }

    // Read position and velocity
    auto [left, right] = odrive->GetFeedback();

    pos_array[0] = -left.position * meters_per_turn;
    pos_array[1] = right.position * meters_per_turn;
    vel_array[0] = -left.velocity * meters_per_turn;
    vel_array[1] = right.velocity * meters_per_turn;
}

ODriveController::~ODriveController()
{
    delete odrive;
}

// Install the module. This code is executed during start-up.

static InitClass init("ODriveController", &ODriveController::Create, "Source/UserModules/EpiMove/ODriveController/");
