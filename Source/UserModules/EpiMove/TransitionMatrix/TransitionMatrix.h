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


#ifndef TransitionMatrix_
#define TransitionMatrix_

#include "IKAROS.h"

class TransitionMatrix: public Module
{
public:
    static Module * Create(Parameter * p) { return new TransitionMatrix(p); }

    TransitionMatrix(Parameter * p) : Module(p) {}
    ~TransitionMatrix();

    void        SetSizes();
    void 		Init();
    void 		Tick();
    void        Command(std::string s, float x, float y, std::string value);

    // Gamma discount factor
    float gamma = 0.95;

    // Input arrays

    float ** grid_matrix;
    int grid_matrix_size_x;
    int grid_matrix_size_y;

    // Output arrays

    float ** transition_matrix;
    int transition_matrix_size_x;
    int transition_matrix_size_y;

    float ** sucessor_representation;
    int sucessor_representation_size_x;
    int sucessor_representation_size_y;

    std::pair<int, int> reward_position = {0, 0};
    float ** reward_gradient;
    int reward_gradient_size_x;
    int reward_gradient_size_y;

    // Helper array
    float ** identity;
    float ** intermediate;

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

    float get_grid_neighbour(int row, int col, Direction direction = NONE);
    int get_node_number(int row, int col, Direction direction = NONE);
    bool is_valid_node(int row, int col, Direction direction = NONE);

};

#endif

