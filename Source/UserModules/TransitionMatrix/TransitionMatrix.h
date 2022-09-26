//
//	MinimalModule.h		This file is a part of the IKAROS project
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


#ifndef LidarSensor_
#define LidarSensor_

#include "IKAROS.h"

class TransitionMatrix: public Module
{
public:
    static Module * Create(Parameter * p) { return new TransitionMatrix(p); }

    TransitionMatrix(Parameter * p) : Module(p) {}
    ~TransitionMatrix();

    void 		Init();
    void 		Tick();

    // Input arrays

    float ** grid_matrix;
    int grid_matrix_size_x;
    int grid_matrix_size_y;

    // Output arrays

    float ** transition_matrix;
    int transition_matrix_size_x;
    int transition_matrix_size_y;

private:

    enum Direction {
        NORTH_EAST = 1,
        NORTH = 2,
        NORTH_WEST = 3,
        EAST = 4,
        WEST = 5,
        SOUTH_EAST = 6,
        SOUTH = 7,
        SOUTH_WEST = 8,
        NONE = -1
    };

    float get_grid_neighbour(int row, int col, Direction direction);
    int get_node_number(int row, int col, Direction direction = NONE);
    bool is_valid_node(int row, int col, Direction direction = NONE);

};

#endif

