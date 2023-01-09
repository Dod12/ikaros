//
//	GridMatrix.cc		This file is a part of the IKAROS project
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

#include "GridMatrix.h"
#include <iostream>

using namespace ikaros;

std::vector<std::pair<int, int>> GridMatrix::interpolate_bresenham(int x0, int y0, float x, float y) {
    std::vector<std::pair<int, int>> points;
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
GridMatrix::Init()
{

    // Get parameters
    Bind(max_distance, "max_distance");

    // Get input arrays
    io(r_array, r_array_size, "R_ARRAY");
    io(theta_array, theta_array_size, "THETA_ARRAY");
    if (r_array_size != theta_array_size) { Notify(msg_fatal_error, "Input arrays must be of equal size"); }

    // Set output matrix
    io(grid_matrix, grid_matrix_size_x, grid_matrix_size_y, "GRID_MATRIX");
    set_matrix(grid_matrix, 0, grid_matrix_size_x, grid_matrix_size_y);

    empty_cells.reserve(grid_matrix_size_x*grid_matrix_size_y/2);
}

void
GridMatrix::Tick()
{       
    set_matrix(grid_matrix, 0, grid_matrix_size_x, grid_matrix_size_y);

    set_matrix(grid_matrix, 0, grid_matrix_size_x, grid_matrix_size_y);

    empty_cells.clear();

    for (int i = 0; i < r_array_size; ++i) {

        float x = r_array[i] * cos(M_PI - theta_array[i]);
        float y = r_array[i] * sin(M_PI - theta_array[i]);

        int x_index = std::clamp((int) ((grid_matrix_size_x / 2) * (x / max_distance) + (grid_matrix_size_x / 2)), 0, grid_matrix_size_x - 1);
        int y_index = std::clamp((int) ((grid_matrix_size_y / 2) * (y / max_distance) + (grid_matrix_size_y / 2)), 0, grid_matrix_size_y - 1);
        grid_matrix[x_index][y_index] = 0; // Setting the grid to 0 means that there is an obstacle in the cell

        auto bresenham_cells = interpolate_bresenham(grid_matrix_size_x / 2, grid_matrix_size_y / 2, x_index, y_index);

        empty_cells.insert(empty_cells.end(), bresenham_cells.begin(), bresenham_cells.end());
    }

    for (auto&& [x, y] : empty_cells) {
        grid_matrix[x][y] = 1;
    }
}

GridMatrix::~GridMatrix()
{
    
}

static InitClass init("GridMatrix", &GridMatrix::Create, "Source/UserModules/EpiMove/GridMatrix/");


