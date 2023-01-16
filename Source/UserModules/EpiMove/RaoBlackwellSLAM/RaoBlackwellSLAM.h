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


struct LidarSensor
{
    float x_offset, y_offset;
    float angle_offset;
    float max_range;
};

struct Pose
{
    float x;
    float y;
    float angle;
};

struct VelocityNoise
{
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;
    float alpha5;
    float alpha6;
};

struct OdometryNoise
{
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;
};

class Particle
{
public:
    float ** grid;
    float grid_size_x;
    float grid_size_y;
    float cell_size;

    Pose pose;

    float weight;

    Particle(float ** grid, float grid_size_x, float grid_size_y, float cell_size, Pose pose, float weight, LidarSensor sensor, float l_0, float l_occ, float l_free, float z_hit, float z_short, float z_max, float z_rand, VelocityNoise vel_noise, float sigma_hit, float lambda_short);

    Particle operator=(const Particle& other);

    // Samples the motion model and returns the new pose
    Pose& sample_motion_model_velocity(float d_t, float velocity, float omega, std::default_random_engine& generator);

    // Samples the motion model and returns the new pose based on the odometry
    Pose& sample_motion_model_odometry(float d_x, float d_y, float wheelbase, std::default_random_engine& generator);

    // Calculates probability of current Lidar scan given the current pose and the old map
    float measurement_model(float * r_array, float * theta_array, int array_size);

    // Iterate over scan beams and update grid cells
    void update_map(float * r_array, float * theta_array, int array_size, float alpha = 0.1f);

private:

    LidarSensor sensor;

    float l_0;
    float l_occ;
    float l_free;

    float z_hit = 0.8f;
    float z_short = 0.1f;
    float z_max = 0.05f;
    float z_rand = 0.05f;

    VelocityNoise vel_noise; // Noise parameters for the velocity model

    float sigma_hit = 0.1; // Standard deviation of the Gaussian distribution for the hit model
    float lambda_short = 0.1; // Decay rate of the exponential distribution for the short model

    // Ray cast on the map to find the "true" distanace to the obstacle in the direction of the Lidar beam
    float ray_cast(float angle);

    // Proboabilities for the measurement model

    float prob_hit(float z_star, float z);
    float prob_short(float z_star, float z);
    float prob_max(float z_star, float z);
    float prob_rand(float z_star, float z);
};

class RaoBlackwellSLAM: public Module
{
public:
    static Module * Create(Parameter * p) { return new RaoBlackwellSLAM(p); }

    RaoBlackwellSLAM(Parameter * p) : Module(p) {}
    ~RaoBlackwellSLAM();

    void 		Init();
    void 		Tick();

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

    float * n_samples;
    int n_samples_size = 1;

    float * pos_estim;
    int pos_estim_size;

    float * prev_pos_estim;

    float * vel_estim;
    int vel_estim_size;

    // Structs
    Pose robot_location;
    LidarSensor sensor;

    std::vector<float **> maps;
    std::vector<Particle> particles;
    std::vector<Particle> new_particles;

    std::vector<float> weights;

    std::random_device random_device;
    std::default_random_engine generator;
    std::discrete_distribution<int> distribution;

    // Output arrays
    float ** occupancy_map;
    int occupancy_map_size_x;
    int occupancy_map_size_y;

    float * heading;
    int heading_size;

    float * position;
    int position_size;

    float * position_index;
    int position_index_size;

private:
};

#endif

