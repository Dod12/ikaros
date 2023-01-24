//
//	RoutePlanner.cc		This file is a part of the IKAROS project
//
//    Copyright (C) 2023 Daniel Carlström Schad
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

#include "RoutePlanner.h"

using namespace ikaros;

void RoutePlanner::SetSizes()
{
    // Set output matrix sizes
    int sizex = GetInputSizeX("SR_GRADIENT");
    int sizey = GetInputSizeY("SR_GRADIENT");
    SetOutputSize("ROUTE", sizex, sizey);
}

void
RoutePlanner::Init()
{
    // Get input arrays
    io(sr_gradient, sr_gradient_size_x, sr_gradient_size_y, "SR_GRADIENT");
    io(target_position, target_position_size_x, "TARGET_POSITION");
    io(start_position, start_position_size_x, "START_POSITION");

    set_array(start_position, 0, start_position_size_x); 

    // Set output matrix
    io(route, route_size_x, route_size_y, "ROUTE");
}

void
RoutePlanner::Tick()
{   
    int pos_x, pos_y;
    if (start_position != nullptr)
    {
        pos_x = start_position[1];
        pos_y = start_position[0];  
    } else {
        pos_x = start_pos_x;
        pos_y = start_pos_y;
    }
        
    set_matrix(route, 0, route_size_x, route_size_y);

    int counter = 0;

    while (counter < route_size_x * route_size_y) {
        route[pos_y][pos_x] = 1;
        float max = 0;
        int max_x = 0, max_y = 0;

        if (pos_x == (int) target_position[0] && pos_y == (int) target_position[1]) {
            break;
        }

        for (auto i : {-1, 0, 1}) {
            for (auto j : {-1, 0, 1}) {
                if (pos_x + i < 0 || pos_x + i >= sr_gradient_size_x || pos_y + j < 0 || pos_y + j >= sr_gradient_size_y) { // Check bounds
                    continue;
                } else if (i == 0 && j == 0) { // Skip current position
                    continue;
                }
                if (sr_gradient[pos_y + j][pos_x + i] > max) {
                    max = sr_gradient[pos_y + j][pos_x + i];
                    max_x = i;
                    max_y = j;
                }
            }
        }

        pos_x += max_x;
        pos_y += max_y;
        ++counter;
    }
}

void RoutePlanner::Command(std::string s, float x, float y, std::string value)
{
    printf("Command: %s %f %f %s", s.c_str(), x, y, value.c_str());
    if (s == "set_position") {
        start_pos_x = x;
        start_pos_y = y;
    }
}

RoutePlanner::~RoutePlanner()
{

}

static InitClass init("RoutePlanner", &RoutePlanner::Create, "Source/UserModules/EpiMove/RoutePlanner/");


