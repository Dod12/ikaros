//
//	MinimalModule.cc		This file is a part of the IKAROS project
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

#include "TransitionMatrix.h"

using namespace ikaros;

void
TransitionMatrix::Init()
{

    // Get input arrays
    io(grid_matrix, grid_matrix_size_x, grid_matrix_size_y, "OBSTACLE_GRID");
    grid_matrix = {}; // Init matrix to 0

    // Set output matrix
    io(transition_matrix, transition_matrix_size_x, transition_matrix_size_y, "TRANSITION_MATRIX");
}

void
TransitionMatrix::Tick()
{
    Direction directions[] = {NORTH_EAST, NORTH, NORTH_WEST, EAST, WEST, SOUTH_EAST, SOUTH, SOUTH_WEST};
    for (int i = 0; i < grid_matrix_size_x; ++i) {
        for (int j = 0; j < grid_matrix_size_y; ++j) {
            int sum = 0;
            for (auto direction : directions) {
                if (is_valid_node(i, j, direction)) {
                    sum += get_grid_neighbour(i, j, direction);
                }
            }
            for (auto direction : directions) {
                if (is_valid_node(i, j, direction)) {
                    transition_matrix[get_node_number(i, j)][get_node_number(i, j, direction)] = get_grid_neighbour(i, j, direction) / sum;
                }
            }
        }
    }
}

float TransitionMatrix::get_grid_neighbour(int row, int col, TransitionMatrix::Direction direction) {
    switch (direction) {
        case TransitionMatrix::Direction::NORTH:
            return grid_matrix[row - 1][col];
        case TransitionMatrix::Direction::NORTH_EAST:
            return grid_matrix[row - 1][col + 1];
        case TransitionMatrix::Direction::EAST:
            return grid_matrix[row][col + 1];
        case TransitionMatrix::Direction::SOUTH_EAST:
            return grid_matrix[row + 1][col + 1];
        case TransitionMatrix::Direction::SOUTH:
            return grid_matrix[row + 1][col];
        case TransitionMatrix::Direction::SOUTH_WEST:
            return grid_matrix[row + 1][col - 1];
        case TransitionMatrix::Direction::WEST:
            return grid_matrix[row][col - 1];
        case TransitionMatrix::Direction::NORTH_WEST:
            return grid_matrix[row - 1][col - 1];
        default:
            return -1;
    }
}

int TransitionMatrix::get_node_number(int row, int col, TransitionMatrix::Direction direction) {
    switch (direction) {
        case TransitionMatrix::Direction::NORTH:
            return (row - 1) * grid_matrix_size_x + col;
        case TransitionMatrix::Direction::NORTH_EAST:
            return (row - 1) * grid_matrix_size_x + col + 1;
        case TransitionMatrix::Direction::EAST:
            return row * grid_matrix_size_x + col + 1;
        case TransitionMatrix::Direction::SOUTH_EAST:
            return (row + 1) * grid_matrix_size_x + col + 1;
        case TransitionMatrix::Direction::SOUTH:
            return (row + 1) * grid_matrix_size_x + col;
        case TransitionMatrix::Direction::SOUTH_WEST:
            return (row + 1) * grid_matrix_size_x + col - 1;
        case TransitionMatrix::Direction::WEST:
            return row * grid_matrix_size_x + col - 1;
        case TransitionMatrix::Direction::NORTH_WEST:
            return (row - 1) * grid_matrix_size_x + col - 1;
        default:
            return row * grid_matrix_size_x + col;
    }
}

bool TransitionMatrix::is_valid_node(int row, int col, TransitionMatrix::Direction direction) {
    switch (direction) {
        case TransitionMatrix::Direction::NORTH:
            return row - 1 >= 0;
        case TransitionMatrix::Direction::NORTH_EAST:
            return row - 1 >= 0 && col + 1 < grid_matrix_size_y;
        case TransitionMatrix::Direction::EAST:
            return col + 1 < grid_matrix_size_y;
        case TransitionMatrix::Direction::SOUTH_EAST:
            return row + 1 < grid_matrix_size_x && col + 1 < grid_matrix_size_y;
        case TransitionMatrix::Direction::SOUTH:
            return row + 1 < grid_matrix_size_x;
        case TransitionMatrix::Direction::SOUTH_WEST:
            return row + 1 < grid_matrix_size_x && col - 1 >= 0;
        case TransitionMatrix::Direction::WEST:
            return col - 1 >= 0;
        case TransitionMatrix::Direction::NORTH_WEST:
            return row - 1 >= 0 && col - 1 >= 0;
        default:
            return row >= 0 && row < grid_matrix_size_x && col >= 0 && col < grid_matrix_size_y;
    }
}

TransitionMatrix::~TransitionMatrix()
{
    
}

static InitClass init("TransitionMatrix", &TransitionMatrix::Create, "Source/UserModules/TransitionMatrix/");


