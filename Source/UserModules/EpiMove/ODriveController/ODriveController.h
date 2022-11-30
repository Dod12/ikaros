//
//	ODriveController.h		This file is a part of the IKAROS project
//
//    Copyright (C) 2012 <Author Name>
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

#ifndef ODriveController_
#define ODriveController_

#include "IKAROS.h"

#include "odrive/odrive.h"

class ODriveController: public Module
{
public:
    static Module * Create(Parameter * p) { return new ODriveController(p); }

    ODriveController(Parameter * p) : Module(p) {}
    virtual ~ODriveController();

    void 		Init();
    void 		Tick();

    // pointers to inputs and outputs
    // and integers to represent their sizes

    float gear_reduction, wheel_circumference, speed;

    int control_mode, input_mode;
    float input_filter_bandwidth, vel_ramp_rate;

    float* target_pos_array;
    int target_pos_array_size;

    float* target_vel_array;
    int target_vel_array_size;

    float* target_torque_array;
    int target_torque_array_size;

    float* pos_array;
    int pos_array_size;

    float* vel_array;
    int vel_array_size;

    odrive::ODrive odrive;
};

#endif
