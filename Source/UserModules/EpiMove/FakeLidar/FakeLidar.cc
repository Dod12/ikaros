//
//	MinimalModule.cc		This file is a part of the IKAROS project
//
//    Copyright (C) 2022 Daniel Carlström Schad
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


#include "FakeLidar.h"
#include <iostream>

using namespace ikaros;

void
FakeLidar::Init()
{

    // Get parameters
    Bind(num_samples, "num_samples");

    io(position, position_size, "POSITION");
    io(heading, heading_size, "HEADING");

    // Set output parameters
    io(r_array, r_array_size, "R_ARRAY");
    io(theta_array, theta_array_size, "THETA_ARRAY");

}

void
FakeLidar::Tick()
{
    int x1 = -5;
    int x2 = 5;
    int y1 = -5;
    int y2 = 5;

    int x = position[0];
    int y = position[1];

    float resolution = 0.01;

    for (int i = 0; i < num_samples; i++)
    {
        float theta = (float)i / (float)num_samples * 2 * M_PI;
        float r = 0;
        float x_step = cos(theta) * resolution;
        float y_step = sin(theta) * resolution;

        while (x > x1 && x < x2 && y > y1 && y < y2)
        {
            x += x_step;
            y += y_step;
            r += resolution;
        }
        theta_array[i] = theta;
        r_array[i] = r;
    }
}

FakeLidar::~FakeLidar()
{

}

static InitClass init("FakeLidar", &FakeLidar::Create, "Source/UserModules/EpiMove/FakeLidar/");


