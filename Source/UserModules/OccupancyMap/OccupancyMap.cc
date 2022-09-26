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

using namespace ikaros;

float prob_to_log_odds(float prob)
{
    return log(prob/(1-prob));
}

std::vector<std::pair<int, int>> interpolate_bresenham(float x, float y) {
    std::vector<std::pair<int, int>> points;
    int x0 = 0, y0 = 0;
    float dx = abs(x - x0);
    float dy = -abs(y - y0);
    int sx = x0 < x ? 1 : -1;
    int sy = y0 < y ? 1 : -1;
    float error = dx + dy;

    while (true) {
        points.push_back(std::make_pair(x0, y0));
        if (x0 == x && y0 == y) break;
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

    // Set output matrix
    io(grid_matrix, grid_matrix_size_x, grid_matrix_size_y, "OCCUPANCY_GRID");
    set_matrix(grid_matrix, l_prior, grid_matrix_size_x, grid_matrix_size_y);
}

void
OccupancyMap::Tick()
{   
    if (r_array_size != theta_array_size) { Notify(msg_fatal_error, "Input arrays must be of equal size"); }
    
    std::vector<std::pair<int, int>> empty_cells;
    empty_cells.reserve(grid_matrix_size_x*grid_matrix_size_y);

    for (int i = 0; i < r_array_size; ++i) {
        float x = r_array[i] * cos(theta_array[i]), y = r_array[i] * sin(theta_array[i]);

        int x_index = std::clamp((int) ((grid_matrix_size_x / 2) * (r_array[i] * cos(theta_array[i]) / max_distance) + (grid_matrix_size_x / 2)), 0, grid_matrix_size_x);
        int y_index = std::clamp((int) ((grid_matrix_size_y / 2) * (r_array[i] * sin(theta_array[i]) / max_distance) + (grid_matrix_size_y / 2)), 0, grid_matrix_size_y);
        grid_matrix[x_index][y_index] += l_occupied - l_prior; // Setting the grid to 1 means that there is an obstacle in the cell

        auto bresenham_cells = interpolate_bresenham(x_index, y_index);

        empty_cells.insert(empty_cells.end(), bresenham_cells.begin(), bresenham_cells.end());
    }

    for (auto&& [x, y] : empty_cells) {
        grid_matrix[x][y] = l_empty - l_prior; // Setting the grid to 0 means that there is no obstacle in the cell
    }
}

OccupancyMap::~OccupancyMap()
{
    
}

static InitClass init("OccupancyMap", &OccupancyMap::Create, "Source/UserModules/OccupancyMap/");


