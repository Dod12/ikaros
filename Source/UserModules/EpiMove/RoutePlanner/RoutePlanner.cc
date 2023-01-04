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
    SetInputSize("TARGET_POSITION", 2);
}

void
RoutePlanner::Init()
{
    // Get input arrays
    io(sr_gradient, sr_gradient_size_x, sr_gradient_size_y, "SR_GRADIENT");
    io(target_position, target_position_size_x, "TARGET_POSITION");

    // Set output matrix
    io(route, route_size_x, route_size_y, "ROUTE");

    start_pos_x = sr_gradient_size_x / 2;
    start_pos_y = sr_gradient_size_y / 2;
}

void
RoutePlanner::Tick()
{   
    set_matrix(route, 0, route_size_x, route_size_y);

    int pos_x = start_pos_x;
    int pos_y = start_pos_y;

    while (true) {
        route[pos_y][pos_x] = 1;
        float max = 0;
        int max_x = 0, max_y = 0;

        for (int i = -1; i < 2; i++) {
            for (int j = -1; j < 2; j++) {
                if (pos_x + i < 0 || pos_x + i >= sr_gradient_size_x || pos_y + j < 0 || pos_y + j >= sr_gradient_size_y) { // Check bounds
                    continue;
                }
                if (sr_gradient[pos_y + j][pos_x + i] > max) {
                    max = sr_gradient[pos_y + j][pos_x + i];
                    max_x = i;
                    max_y = j;
                }
            }
        }

        if (max_x == 0 && max_y == 0 && pos_x == (int) target_position[0] && pos_y == (int) target_position[1]) {
            break;
        }

        pos_x += max_x;
        pos_y += max_y;
    }
}

void RoutePlanner::Command(std::string s, float x, float y, std::string value)
{
    if (s == "set_position") {
        start_pos_x = (int) x;
        start_pos_y = (int) y;
    }
}

RoutePlanner::~RoutePlanner()
{

}

static InitClass init("RoutePlanner", &RoutePlanner::Create, "Source/UserModules/EpiMove/RoutePlanner/");


