//
//	RaoBlackwellSLAM.h		This file is a part of the IKAROS project
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


#ifndef RaoBlackwellSLAM_
#define RaoBlackwellSLAM_

#include <random>
#include "IKAROS.h"

// Log odds helper functions
template <typename T>
static constexpr T log_odds(T p)
{
    return log(p/(1-p));
}

template <typename T>
static constexpr T prob(T l)
{
    return 1/(1+exp(-l));
}

struct RobotLocation
{
    float x;
    float y;
    float angle;
};

struct LidarSensor
{
    float x_offset, y_offset;
    float angle_offset;
    float max_range;
};

struct GridMap
{
    float ** data;
    size_t size_x;
    size_t size_y;
    float cell_size;
    bool destroy_map;
    float l_0;

    GridMap(float ** map, size_t size_x, size_t size_y, float cell_size) : data(map), size_x(size_x), size_y(size_y), cell_size(cell_size), destroy_map(false) {}
    GridMap(size_t size_x, size_t size_y, float cell_size, float l_0) : size_x(size_x), size_y(size_y), cell_size(cell_size), destroy_map(true)
    {
        data = create_matrix(size_x, size_y);
        set_matrix(data, l_0, size_x, size_y);
    }
    GridMap(const GridMap & other) : size_x(other.size_x), size_y(other.size_y), cell_size(other.cell_size), destroy_map(true) {
        data = create_matrix(size_x, size_y);
        copy_matrix(data, other.data, size_x, size_y);
    }

    ~GridMap()
    {
        if (destroy_map)
            destroy_matrix(data);
    }

    float * operator[](size_t x) { return data[x]; }

    size_t GetXIndex(float x) const { return std::clamp((size_t) (x / cell_size) + size_x / 2, (unsigned long) 0, (unsigned long) size_x); }
    size_t GetYIndex(float y) const { return std::clamp((size_t) (y / cell_size) + size_y / 2, (unsigned long) 0, (unsigned long) size_y); }

    float GetXCoordinate(size_t x) const { return x * cell_size + cell_size/2; }
    float GetYCoordinate(size_t y) const { return y * cell_size + cell_size/2; }

    std::pair<int, int> GetClosestOccupiedCell(float x_pos, float y_pos) const 
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
                        if (data[x_index + x][y_index + y] >= log_odds(0.65))
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

    float GetClosestOccupiedDistance(float x_pos, float y_pos) const
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
                        if (data[x_index + x][y_index + y] >= log_odds(0.65))
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

struct Particle
{
    RobotLocation location;
    float weight;
    GridMap map;

    explicit Particle(RobotLocation location, GridMap* map) : location(location), map(*map) {}

    Particle(RobotLocation location, GridMap map) : location(location), map(map) {}
    
    Particle(RobotLocation location, float weight, GridMap map) : location(location), weight(weight), map(map) {}
};

class RaoBlackwellSLAM: public Module
{
public:
    static Module * Create(Parameter * p) { return new RaoBlackwellSLAM(p); }

    RaoBlackwellSLAM(Parameter * p) : Module(p) {}
    ~RaoBlackwellSLAM();

    void 		Init();
    void 		Tick();

    RobotLocation 	sample_motion_model_velocity(RobotLocation& location, float d_t, float velocity, float omega);
    RobotLocation 	sample_motion_model_odometry(RobotLocation& location);

    float           measurement_model_likelihood(const RobotLocation& location, const GridMap& map);
    float           measurement_model_beam_range(const RobotLocation& location, const GridMap& map);

    float           inverse_sensor_model(float x_coord, float y_coord, const RobotLocation& location);

    GridMap&        update_map(GridMap& map, const RobotLocation& location);

    // Parameters
    float wheelbase;

    size_t num_particles;

    float cell_size;
    
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;
    float alpha5;
    float alpha6;
    
    float z_hit;
    float z_short;
    float z_max;
    float z_rand;
    
    float sigma_hit;

    float l_0;
    float l_occupied;
    float l_free;

    float alpha;
    float beta;

    // Input arrays

    float * r_array;
    int r_array_size;

    float * theta_array;
    int theta_array_size;

    float * pos_estim;
    int pos_estim_size;

    float * vel_estim;
    int vel_estim_size;

    // Structs
    RobotLocation robot_location;
    LidarSensor sensor;

    std::vector<Particle> particles;
    std::vector<Particle> new_particles;

    std::vector<float> weights;

    std::random_device random_device;
    std::mt19937 generator;
    std::discrete_distribution<int> distribution;

    // Output arrays
    float ** occupancy_map;
    int occupancy_map_size_x;
    int occupancy_map_size_y;

    float * heading;
    int heading_size;

    float * position;
    int position_size;

private:
};

#endif

