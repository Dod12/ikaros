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

struct RobotLocation
{
    float x;
    float y;
    float angle;
};

struct LidarSensor
{
    float x_offset, y_offset;
    float theta_offset;
    float max_distance;
};

struct Map
{
    float ** map;
    size_t size_x;
    size_t size_y;
    float cell_size;
    bool destroy_map;

    Map(float ** map, size_t size_x, size_t size_y, float cell_size) : map(map), size_x(size_x), size_y(size_y), cell_size(cell_size), destroy_map(false) {}
    Map(size_t size_x, size_t size_y, float cell_size) : size_x(size_x), size_y(size_y), cell_size(cell_size), destroy_map(true)
    {
        map = create_matrix(size_x, size_y);
    }

    ~Map()
    {
        if (destroy_map)
            destroy_matrix(map);
    }

    float * operator[](size_t x) { return map[x]; }

    size_t GetXIndex(float x) { return (size_t) (x / cell_size); }
    size_t GetYIndex(float y) { return (size_t) (y / cell_size); }

    float GetXCoordinate(size_t x) { return x * cell_size; }
    float GetYCoordinate(size_t y) { return y * cell_size; }

    std::pair<int, int> GetClosestOccupiedCell(float x_pos, float y_pos)
    {   
        size_t x_index = GetXIndex(x_pos);
        size_t y_index = GetYIndex(y_pos);

        for (int r = 1; r < size_x; ++r)
        {
            int closest_x = -1, closest_y = -1;
            float closest_distance;
            
            for (int x = -r; x <= r; ++x)
            {
                for (int y = -r; y <= r; ++y)
                {
                    if ((abs(x) == r || abs(y) == r) && x_index + x >= 0 && x_index + x < size_x && y_index + y >= 0 && y_index + y < size_y)
                    {
                        if (map[x_index + x][y_index + y] == 1)
                        {
                            float distance = sqrt(pow(x_pos - GetXCoordinate(x_index + x), 2) + pow(y_pos - GetYCoordinate(y_index + y), 2));
                            if (distance < closest_distance)
                            {
                                closest_x = x_index + x;
                                closest_y = y_index + y;
                                closest_distance = distance;
                            }
                        }
                    }
                }
            }
            if (closest_x != -1 || closest_y != -1)
                return std::make_pair(closest_x, closest_y);
        }
        return std::make_pair(-1, -1);
    }

    float GetClosestOccupiedDistance(float x_pos, float y_pos)
    {
        size_t x_index = GetXIndex(x_pos);
        size_t y_index = GetYIndex(y_pos);

        for (int r = 1; r < size_x; ++r)
        {
            int closest_x = -1, closest_y = -1;
            float closest_distance;
            
            for (int x = -r; x <= r; ++x)
            {
                for (int y = -r; y <= r; ++y)
                {
                    if ((abs(x) == r || abs(y) == r) && x_index + x >= 0 && x_index + x < size_x && y_index + y >= 0 && y_index + y < size_y)
                    {
                        if (map[x_index + x][y_index + y] == 1)
                        {
                            float distance = sqrt(pow(x_pos - GetXCoordinate(x_index + x), 2) + pow(y_pos - GetYCoordinate(y_index + y), 2));
                            if (distance < closest_distance)
                            {
                                closest_x = x_index + x;
                                closest_y = y_index + y;
                                closest_distance = distance;
                            }
                        }
                    }
                }
            }
            if (closest_x != -1 || closest_y != -1)
                return closest_distance;
        }
        return -1;
    }
};

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
    std::vector<std::pair<int, int>> empty_cells_ego;

private:
};

#endif

