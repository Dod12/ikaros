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
    Bind(circumference, "wheel_circumference");

    io(encoder_counts, encoder_counts_size, "ENCODER_COUNTS");

    io(position, position_size, "POSITION");
    set_array(position, 0, position_size);
    io(heading, heading_size, "HEADING");
    set_array(heading, 0, heading_size);

    prev_encoder_counts = create_array(encoder_counts_size);
    set_array(prev_encoder_counts, 0, encoder_counts_size);

    rotation_centre = create_array(position_size);
}

void
PathIntegrator::Tick()
{
    if (ticks < 3) ++ticks; // Skip first few interations

    float n_l = circumference * (encoder_counts[0] - prev_encoder_counts[0]) / (float) 8192;
    float n_r = circumference * (encoder_counts[1] - prev_encoder_counts[1]) / (float) 8192;
    
    if ((abs(n_r) > 1e-2 || abs(n_l) > 1e-2) && ticks > 2 ){ // Only update position if significant movement AND some ticks have passed
        float R = (wheelbase / 2) * (n_l + n_r) / (n_r - n_l);
        float omega_delta_t =  (n_r + n_l) / wheelbase;
        if (abs(R) > 25) { // If the radius of the arc is larger than 25 m, we can assume straight motion.
            std::cout << "Moving straight" << std::endl;
            float movement = (n_r + n_l) / 2;
            position[0] += movement * cos(heading[0]);
            position[1] += movement * sin(heading[0]);
        } else { // Calculate arclength 
            std::cout << "Moving in arc" << std::endl;
            
            rotation_centre[0] = position[0] - R * sin(heading[0]);
            rotation_centre[1] = position[1] + R * cos(heading[0]);

            float new_x = cos(omega_delta_t) * (position[0] - rotation_centre[0]) - sin(omega_delta_t) * (position[1] - rotation_centre[1]) + rotation_centre[0];
            float new_y = sin(omega_delta_t) * (position[0] - rotation_centre[0]) + cos(omega_delta_t) * (position[1] - rotation_centre[1]) + rotation_centre[1];
            position[0] = new_x;
            position[1] = new_y;
            heading[0] += omega_delta_t;
/*
            r = (wheelbase / 2) * (right_delta + left_delta) / (right_delta - left_delta);
            omega = (right_delta - left_delta) / wheelbase;
            std::cout << "Omega: " << omega << std::endl;

            position[0] += r * (sin(omega + heading[0]) - sin(heading[0]));
            position[1] -= r * (cos(omega + heading[0]) - cos(heading[0]));
            heading[0] += omega; // TODO: FIX ME
*/
            /*
            

            rotation_centre[0] = position_turns[0] - r * sin(heading[0]);
            rotation_centre[1] = position_turns[1] + r * cos(heading[0]);

            float x_position = rotation_centre[0] + (position_turns[0] - rotation_centre[0]) * cos(omega) - (position_turns[1] - rotation_centre[1]) * sin(omega);
            float y_position = rotation_centre[1] + (position_turns[0] - rotation_centre[0]) * sin(omega) + (position_turns[1] - rotation_centre[1]) * cos(omega);
            position_turns[0] = x_position; position_turns[1] = y_position;
            heading[0] = heading[0] + omega;
            */
        }
        std::cout << "OUTPUT: X: " << position[0] << ", Y: " << position[1] << ", Heading: " << heading[0] << std::endl;
        prev_encoder_counts[0] = encoder_counts[0];
        prev_encoder_counts[1] = encoder_counts[1];
    } else {
        // No movement
        prev_encoder_counts[0] = encoder_counts[0];
        prev_encoder_counts[1] = encoder_counts[1];
    }
}

PathIntegrator::~PathIntegrator()
{
    destroy_array(prev_encoder_counts);
    destroy_array(rotation_centre);
}

// Install the module. This code is executed during start-up.

static InitClass init("PathIntegrator", &PathIntegrator::Create, "Source/UserModules/PathIntegrator/");
