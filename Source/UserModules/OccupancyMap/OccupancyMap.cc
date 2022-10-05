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

#include "OccupancyMap.h"
#include <iostream>

using namespace ikaros;

float prob_to_log_odds(float prob)
{
    return log(prob/(1-prob));
}

std::vector<std::pair<int, int>> interpolate_bresenham(float x, float y) {
    std::vector<std::pair<int, int>> points;
    int x0 = 128, y0 = 128; // Centre of gridmap is at half the size.
    float dx = abs(x - x0);
    float dy = -abs(y - y0);
    int sx = x0 < x ? 1 : -1;
    int sy = y0 < y ? 1 : -1;
    float error = dx + dy;

    while (true) {
        if (x0 == x && y0 == y) break;
        points.push_back(std::make_pair(x0, y0));
        float e2 = 2 * error;
        if (e2 >= dy) {
            if (x0 == x) break;
            error += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            if (y0 == y) break;
            error += dx;
            y0 += sy;
        }
    }
    return points;
}

void
OccupancyMap::Init()
{

    // Get parameters
    Bind(max_distance, "max_distance");
    Bind(prior, "prior");
    l_prior = prob_to_log_odds(prior);
    Bind(empty_probability, "empty_probability"); 
    l_empty = prob_to_log_odds(empty_probability);
    Bind(occupied_probability, "occupied_probability"); 
    l_occupied = prob_to_log_odds(occupied_probability);

    // Get input arrays
    io(r_array, r_array_size, "R_ARRAY");
    io(theta_array, theta_array_size, "THETA_ARRAY");
    io(position, position_size, "POSITION");
    io(heading, heading_size, "HEADING");

    // Set output matrix
    io(grid_matrix, grid_matrix_size_x, grid_matrix_size_y, "GRID_MATRIX");
    set_matrix(grid_matrix, 0, grid_matrix_size_x, grid_matrix_size_y);
    io(occupancy_matrix, occupancy_matrix_size_x, occupancy_matrix_size_y, "OCCUPANCY_MATRIX");
    set_matrix(occupancy_matrix, 0, occupancy_matrix_size_x, occupancy_matrix_size_y);

    empty_cells.reserve(occupancy_matrix_size_x*occupancy_matrix_size_y/2);
}

void
OccupancyMap::Tick()
{   
    if (r_array_size != theta_array_size) { Notify(msg_fatal_error, "Input arrays must be of equal size"); }
    
    set_matrix(grid_matrix, 0, grid_matrix_size_x, grid_matrix_size_y);

    empty_cells.clear();

    for (int i = 0; i < r_array_size; ++i) {
        float x_egocentric = r_array[i] * cos(theta_array[i]);
        float y_egocentric = r_array[i] * sin(theta_array[i]);

        float x = x_egocentric*cos(heading[0]) - y_egocentric*sin(heading[0]) + position[0];
        float y = x_egocentric*sin(heading[0]) + y_egocentric*cos(heading[0]) + position[1];

        if (x >= max_distance) {
            //std::cout << "X larger than max: " << x << std::endl;
        } else if (y >= max_distance) {
            //std::cout << "Y larger than max: " << y << std::endl;
        }

        //std::cout << "r: " << r_array[i] << ", theta: " << theta_array[i] << std::endl;

        int x_index = std::clamp((int) ((occupancy_matrix_size_x / 2) * (x / max_distance) + (occupancy_matrix_size_x / 2)), 0, occupancy_matrix_size_x - 1);
        int y_index = std::clamp((int) ((occupancy_matrix_size_y / 2) * (y / max_distance) + (occupancy_matrix_size_y / 2)), 0, occupancy_matrix_size_y - 1);
        occupancy_matrix[x_index][y_index] += l_occupied - l_prior;
        grid_matrix[x_index][y_index] = 1; // Setting the grid to 1 means that there is an obstacle in the cell

        auto bresenham_cells = interpolate_bresenham(x_index, y_index);

        empty_cells.insert(empty_cells.end(), bresenham_cells.begin(), bresenham_cells.end());
    }

    for (auto&& [x, y] : empty_cells) {
        occupancy_matrix[x][y] += l_empty - l_prior;
    }
}

OccupancyMap::~OccupancyMap()
{
    
}

static InitClass init("OccupancyMap", &OccupancyMap::Create, "Source/UserModules/OccupancyMap/");


