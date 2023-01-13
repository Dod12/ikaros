//
//	RaoBlackwellSLAM.cc		This file is a part of the IKAROS project
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

#include "RaoBlackwellSLAM.h"
#include <iostream>

using namespace ikaros;

// Normal distribution helper functions
template <typename T>
static constexpr T prob_normal_distribution(T a, T b2)
{
    return exp(-pow(a,2)/(2*b2))/sqrt(2*M_PI*b2);
}

template <typename T>
static constexpr T sample_normal_distribution(T a, T b2)
{
    float acc = 0;
    for (int i = 0; i < 12; ++i)
        acc += (float) random(-b2,b2);
    return a + 0.5 * acc;
}

// Variadic min
template <typename T>
static constexpr T min(T a)
{
    return a;
}

template <typename First, typename... Rest>
constexpr First min(First a, const Rest&... rest)
{
    return std::min({std::cref(rest)...}, a).get();
}

void
RaoBlackwellSLAM::Init()
{
    Bind(wheelbase, "wheelbase");
    
    int num_particles_int;
    Bind(num_particles_int, "num_particles");
    num_particles = (size_t) num_particles_int;

    Bind(cell_size, "cell_size");

    alpha1 = 0.1;
    alpha2 = 0.1;
    alpha3 = 0.1;
    alpha4 = 0.1;
    alpha5 = 0.1;
    alpha6 = 0.1;

    Bind(z_hit, "z_hit");
    Bind(z_short, "z_short");
    Bind(z_max, "z_max");
    Bind(z_rand, "z_rand");

    Bind(sigma_hit, "sigma_hit");

    float prob_0, prob_occupied, prob_free;
    Bind(prob_0, "p_unknown");
    Bind(prob_occupied, "p_occupied");
    Bind(prob_free, "p_free");

    l_0 = log_odds(prob_0);
    l_occupied = log_odds(prob_occupied);
    l_free = log_odds(prob_free);

    Bind(alpha, "alpha");
    Bind(beta, "beta");

    float max_range, x_offset, y_offset, angle_offset;
    Bind(max_range, "lidar_max_range");
    Bind(x_offset, "lidar_x_offset");
    Bind(y_offset, "lidar_y_offset");
    Bind(angle_offset, "lidar_angle_offset");

    generator = std::mt19937(random_device());
    distribution = std::discrete_distribution<int>();

    // Input arrays

    io(r_array, r_array_size, "R_ARRAY");
    io(theta_array, theta_array_size, "THETA_ARRAY");

    io(pos_estim, pos_estim_size, "POS_ESTIM");
    io(vel_estim, vel_estim_size, "VEL_ESTIM");

    // Output arrays
    io(heading, heading_size, "HEADING");
    io(position, position_size, "POSITION");

    io(occupancy_map, occupancy_map_size_x, occupancy_map_size_y, "OCCUPANCY_MAP");

    // Initialize structs

    sensor = LidarSensor();
    sensor.angle_offset = angle_offset;
    sensor.x_offset = x_offset;
    sensor.y_offset = y_offset;
    sensor.max_range = max_range;

    robot_location = RobotLocation();
    robot_location.x = 0;
    robot_location.y = 0;
    robot_location.angle = 0;

    particles.reserve(num_particles);
    new_particles.reserve(num_particles);
    weights.reserve(num_particles);

    GridMap map(occupancy_map_size_x, occupancy_map_size_y, cell_size, l_0);
    // update_map(map, robot_location);

    for (int i = 0; i < num_particles; ++i)
    {   
        particles.push_back(Particle(robot_location, map));
    }

}

void
RaoBlackwellSLAM::Tick()
{       
    if (ticks == 0)
    {
        for (auto& particle : particles)
            particle.map = update_map(particle.map, particle.location);
    }

    new_particles.clear();
    weights.clear();

    for (auto& particle : particles)
    {
        particle.location = sample_motion_model_odometry(particle.location);
        weights.push_back(measurement_model_likelihood(particle.location, particle.map));
        particle.map = std::move(update_map(particle.map, particle.location));
        new_particles.push_back(std::move(particle));
    }

    distribution = std::discrete_distribution<int>(weights.begin(), weights.end());
    particles.clear();

    for (int i = 0; i < num_particles; ++i)
    {
        particles.emplace_back(new_particles[distribution(generator)]);
    }
    for (int i = 0; i < occupancy_map_size_x; ++i)
    {
        for (int j = 0; j < occupancy_map_size_y; ++j)
        {
            occupancy_map[i][j] = prob(particles[0].map.data[i][j]);
        }
    }
    heading[0] = particles[0].location.angle;
    position[0] = particles[0].location.x;
    position[1] = particles[0].location.y;
}

RobotLocation RaoBlackwellSLAM::sample_motion_model_velocity(RobotLocation& location, float d_t, float velocity, float omega)
{
    float v_hat = velocity + sample_normal_distribution((float) 0, alpha1 * velocity * velocity + alpha2 * omega * omega);
    float omega_hat = omega + sample_normal_distribution((float) 0, alpha3 * velocity * velocity + alpha4 * omega * omega);
    float gamma_hat = sample_normal_distribution((float) 0, alpha5 * velocity * velocity + alpha6 * omega * omega);

    location.x = location.x + (v_hat / omega_hat) * (sin(location.angle + omega_hat * d_t) - sin(location.angle));
    location.y = location.y + (v_hat / omega_hat) * (cos(location.angle) - cos(location.angle + omega_hat * d_t));
    location.angle = location.angle + omega_hat * d_t + gamma_hat * d_t;
    return location;
}


RobotLocation RaoBlackwellSLAM::sample_motion_model_odometry(RobotLocation& location)
{   
    float l_l_hat = pos_estim[0] + sample_normal_distribution((float) 0, alpha1 * pos_estim[0] * pos_estim[0] + alpha2 * pos_estim[1] * pos_estim[1]);
    float l_r_hat = pos_estim[1] + sample_normal_distribution((float) 0, alpha3 * pos_estim[0] * pos_estim[0] + alpha4 * pos_estim[1] * pos_estim[1]);
    float r = wheelbase / 2 * (l_l_hat + l_r_hat) / (l_r_hat - l_l_hat);
    float omega_delta_t = (l_r_hat - l_l_hat) / wheelbase;
    float ICC_x = location.x - r * sin(location.angle);
    float ICC_y = location.y + r * cos(location.angle);
    
    location.x = ICC_x + (location.x - ICC_x) * cos(omega_delta_t) - (location.y - ICC_y) * sin(omega_delta_t);
    location.y = ICC_y + (location.x - ICC_x) * sin(omega_delta_t) + (location.y - ICC_y) * cos(omega_delta_t);
    location.angle = location.angle + omega_delta_t;
    return location;
}

float RaoBlackwellSLAM::measurement_model_beam_range(const RobotLocation& location, const GridMap& map)
{
    float q = 1;
    for (int i = 0; i < r_array_size; ++i)
    {
        
    }
    return q;
}

float RaoBlackwellSLAM::measurement_model_likelihood(const RobotLocation& location, const GridMap& map)
{
    double q = 1;
    for (int i = 0; i < r_array_size; ++i)
    {
        if (r_array[i] == 0 || r_array[i] >= sensor.max_range)
            continue;
        
        double z_x = location.x + sensor.x_offset + r_array[i] * cos(location.angle + sensor.angle_offset + theta_array[i]);
        double z_y = location.y + sensor.y_offset + r_array[i] * sin(location.angle + sensor.angle_offset + theta_array[i]);
        float distance = map.GetClosestOccupiedDistance(z_x, z_y);
        q *= z_hit * prob_normal_distribution(distance, sigma_hit) + z_rand / z_max;
    }
    return q;
}

float RaoBlackwellSLAM::inverse_sensor_model(float x_coord, float y_coord, const RobotLocation& location)
{
    float r = sqrt(pow(x_coord - location.x, 2) + pow(y_coord - location.y, 2));
    float phi = atan2(y_coord - location.y, x_coord - location.x) - location.angle;
    int min_phase_index = -1;
    float min_phase_diff = 2 * M_PI;
    for (int i = 0; i < r_array_size; ++i)
    {
        float phase_diff = abs(phi - theta_array[i]+sensor.angle_offset);
        if (phase_diff < min_phase_diff)
        {
            min_phase_diff = phase_diff;
            min_phase_index = i;
        } else if (min_phase_index != -1 && phase_diff > min_phase_diff) // Angles are monotonically increasing, so we can stop if we found a local minimum
        {
            break;
        }
    }
    if (r > min(z_max, r_array[min_phase_index] + alpha/2) || 
        abs(phi - theta_array[min_phase_index]+sensor.angle_offset) > beta/2)
        return l_0;
    else if (r_array[min_phase_index] < z_max && abs(r - r_array[min_phase_index]) < alpha/2)
        return l_occupied;
    else if (r < r_array[min_phase_index])
        return l_free;
    else
        return l_0; // This should never happen
}

GridMap& RaoBlackwellSLAM::update_map(GridMap& map, const RobotLocation& location)
{
    for (int i = 0; i < map.size_x; ++i)
    {
        for (int j = 0; j < map.size_y; ++j)
        {   
            float x_coord = map.GetXCoordinate(i);
            float y_coord = map.GetYCoordinate(j);

            // If cell is in max radius of sensor, update it
            if (sqrt(pow(x_coord - location.x, 2) + pow(y_coord - location.y, 2)) < sensor.max_range)
                map[i][j] = map[i][j] + inverse_sensor_model(x_coord, y_coord, location) - l_0;
        }
    }
    return map;
}

RaoBlackwellSLAM::~RaoBlackwellSLAM()
{
    
}

static InitClass init("RaoBlackwellSLAM", &RaoBlackwellSLAM::Create, "Source/UserModules/EpiMove/RaoBlackwellSLAM/");


