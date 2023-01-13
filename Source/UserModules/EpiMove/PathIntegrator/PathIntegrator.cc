//
//	PathIntegrator.cc		This file is a part of the IKAROS project
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

    io(pos_estim, pos_estim_size, "POS_ESTIM");
    io(vel_estim, vel_estim_size, "VEL_ESTIM");

    io(position, position_size, "POSITION");
    set_array(position, 0, position_size);
    io(heading, heading_size, "HEADING");
    set_array(heading, 0, heading_size);

    prev_values = create_array(pos_estim_size);
    set_array(prev_values, 0, pos_estim_size);

    rotation_centre = create_array(position_size);
}

void
PathIntegrator::Tick()
{       

    float n_l = pos_estim[0] - prev_values[0];
    float n_r = pos_estim[1] - prev_values[1];

    if (abs(n_l) < 1e-3 && abs(n_r) < 1e-3) // Skip updating the path if movements are smaller than a mm.
    {
        prev_values[0] = pos_estim[0];
        prev_values[1] = pos_estim[1];
        return;
    } 

    std::cout << "n_l: " << n_l << ", n_r: " << n_r << std::endl;

    float R = (wheelbase / 2) * (n_l + n_r) / (n_r - n_l);
    float omega_delta_t =  (n_r - n_l) / wheelbase;
    if (abs(R) >= 10) { // If  the radius of the arc is larger than 10 m, we can assume straight motion.
        std::cout << "Moving straight, R: " << R << ", omega: " << omega_delta_t << std::endl;
        float movement = (n_r + n_l) / 2;
        position[0] += movement * cos(heading[0]);
        position[1] += movement * sin(heading[0]);
    } else { // Calculate arclength 
        std::cout << "Moving in arc, R: " << R << ", omega: " << omega_delta_t << std::endl;
        
        rotation_centre[0] = position[0] - R * sin(heading[0]);
        rotation_centre[1] = position[1] + R * cos(heading[0]);

        float new_x = cos(omega_delta_t) * (position[0] - rotation_centre[0]) - sin(omega_delta_t) * (position[1] - rotation_centre[1]) + rotation_centre[0];
        float new_y = sin(omega_delta_t) * (position[0] - rotation_centre[0]) + cos(omega_delta_t) * (position[1] - rotation_centre[1]) + rotation_centre[1];
        position[0] = new_x;
        position[1] = new_y;
        heading[0] = heading[0] - omega_delta_t;
    }
    std::cout << "OUTPUT: X: " << position[0] << ", Y: " << position[1] << ", Heading: " << heading[0] << std::endl;
    prev_values[0] = pos_estim[0];
    prev_values[1] = pos_estim[1];
}

PathIntegrator::~PathIntegrator()
{
    destroy_array(prev_values);
    destroy_array(rotation_centre);
}

// Install the module. This code is executed during start-up.

static InitClass init("PathIntegrator", &PathIntegrator::Create, "Source/UserModules/EpiMove/PathIntegrator/");
