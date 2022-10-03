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


#ifndef OccupancyMap_
#define OccupancyMap_

#include "IKAROS.h"

class OccupancyMap: public Module
{
public:
    static Module * Create(Parameter * p) { return new OccupancyMap(p); }

    OccupancyMap(Parameter * p) : Module(p) {}
    ~OccupancyMap();

    void 		Init();
    void 		Tick();

    // Input arrays

    float * r_array;
    int r_array_size;

    float * theta_array;
    int theta_array_size;

    float * position;
    int position_size;

    float * heading;
    int heading_size;

    // Output arrays

    float ** occupancy_matrix;
    int occupancy_matrix_size_x;
    int occupancy_matrix_size_y;

    float ** grid_matrix;
    int grid_matrix_size_x;
    int grid_matrix_size_y;

    // Parameters

    float max_distance;
    float prior, l_prior, empty_probability, l_empty, occupied_probability, l_occupied;

    std::vector<std::pair<int, int>> empty_cells;

private:
};

#endif

