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

#include "PathIntegrator.h"
#include <iostream>

// use the ikaros namespace to access the math library
// this is preferred to using <cmath>

using namespace ikaros;

void
PathIntegrator::Init()
{
    // To get the parameters from the IKC file, use the Bind
    // function for each parameter. The parameters are initialized
    // from the IKC and can optionally be changed from the
    // user interface while Ikaros is running. If the parameter is not
    // set, the default value specified in the ikc-file will be used instead.

    Bind(wheelbase, "wheelbase");

    io(pos_array, pos_array_size, "POS_ARRAY");

    io(position, position_size, "POSITION");
    set_array(position, 0, position_size);
    io(heading, heading_size, "HEADING");
    set_array(heading, 0, heading_size);

    prev_pos_array = create_array(pos_array_size);
    set_array(prev_pos_array, 0, pos_array_size);
    vel_array = create_array(pos_array_size);
    set_array(vel_array, 0, pos_array_size);

    rotation_centre = create_array(position_size);
}

void
PathIntegrator::Tick()
{   
    std::cout << "NEW POS:  X: " << pos_array[0] << ", Y: " << pos_array[1] << std::endl;
    std::cout << "OLD POS:  X: " << prev_pos_array[0] << ", Y: " << prev_pos_array[1] << std::endl;

    vel_array[0] = pos_array[0] - prev_pos_array[0];
    vel_array[1] = pos_array[1] - prev_pos_array[1];

    std::cout << "VELOCITY: X: " << vel_array[0] << ", Y: " << vel_array[1] << std::endl;
    
    if (abs(vel_array[0]) > 1e-5 || abs(vel_array[1]) > 1e-5){
        r = (wheelbase / 2) * ((vel_array[0] - vel_array[1])/(vel_array[0] + vel_array[1]));
        omega = (vel_array[1] - vel_array[0]) / 2;

        rotation_centre[0] = prev_pos_array[0] - r * sin(heading[0]);
        rotation_centre[1] = prev_pos_array[1] + r * cos(heading[0]);

        position[0] = rotation_centre[0] + (prev_pos_array[0] - rotation_centre[0]) * cos(omega) - (prev_pos_array[1] - rotation_centre[1]) * sin(omega);
        position[1] = rotation_centre[0] + (prev_pos_array[0] - rotation_centre[0]) * sin(omega) + (prev_pos_array[1] - rotation_centre[1]) * cos(omega);
        heading[0] = heading[0] + omega;

        std::cout << "OUTPUT: X: " << position[0] << ", Y: " << position[1] << ", Heading: " << heading[0] << std::endl;

        prev_pos_array[0] = pos_array[0];
        prev_pos_array[1] = pos_array[1];
    } else {
        // No movement
        prev_pos_array[0] = pos_array[0];
        prev_pos_array[1] = pos_array[1];
    }
}

PathIntegrator::~PathIntegrator()
{
    destroy_array(prev_pos_array);
    destroy_array(vel_array);
    destroy_array(rotation_centre);
}

// Install the module. This code is executed during start-up.

static InitClass init("PathIntegrator", &PathIntegrator::Create, "Source/UserModules/PathIntegrator/");
