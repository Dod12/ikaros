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

using namespace ikaros;

void
RaoBlackwellSLAM::Init()
{
    Bind(wheelbase, "wheelbase");
    
    int num_particles_int;
    Bind(num_particles_int, "num_particles");
    num_particles = (size_t) num_particles_int;

    Bind(cell_size, "cell_size");

    VelocityNoise vel_noise{0.1};

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

    generator = std::default_random_engine(random_device());
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

    robot_location = Pose();
    robot_location.x = occupancy_map_size_x * cell_size / 2.0f; // Center of map
    robot_location.y = occupancy_map_size_y * cell_size / 2.0f;
    robot_location.angle = M_PI_2; // Facing up

    particles.reserve(num_particles);
    new_particles.reserve(num_particles);
    weights.reserve(num_particles);

    for (size_t i = 0; i < num_particles; i++)
    {
        maps.push_back(create_matrix(occupancy_map_size_x, occupancy_map_size_y));
        set_matrix(maps[i], occupancy_map_size_x, occupancy_map_size_y, l_0);
        particles.push_back(Particle(maps[i], occupancy_map_size_x, occupancy_map_size_y, cell_size, robot_location, 1 / num_particles, sensor, l_0, l_occupied, l_free, z_hit, z_short, z_max, z_rand, vel_noise, sigma_hit, 0.4));
        weights.push_back(1 / num_particles);
    }

}

void
RaoBlackwellSLAM::Tick()
{       
    if (GetTick() == 0)
    {
        for (auto& particle : particles)
            particle.update_map(r_array, theta_array, r_array_size);
    }

    float velocity = (vel_estim[0] + vel_estim[1]) / 2;
    float omega = (vel_estim[1] - vel_estim[0]) / wheelbase;

    weights.clear();

    // Loop through all particles and update their location and map
    for (auto& particle : particles)
    {
        particle.sample_motion_model(GetTickLength(), velocity, omega, generator);
        weights.push_back(particle.measurement_model(r_array, theta_array, r_array_size));
        particle.update_map(r_array, theta_array, r_array_size);
    }

    // Get best particle
    auto best_particle = std::max_element(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) { return a.weight < b.weight; });
    robot_location = best_particle->pose;
    copy_matrix(occupancy_map, best_particle->grid, occupancy_map_size_x, occupancy_map_size_y);
    position[0] = robot_location.x;
    position[1] = robot_location.y;
    heading[0] = robot_location.angle;

    // Resample
    float N_eff = 1 / std::accumulate(weights.begin(), weights.end(), 0.0f, [](float a, float b) { return a + b * b; });

    if (N_eff < num_particles / 2)
    {
        distribution = std::discrete_distribution<int>(weights.begin(), weights.end());
        new_particles = std::move(particles);
        particles.clear();
        for (size_t i = 0; i < num_particles; i++)
        {
            int index = distribution(generator);
            particles.push_back(std::move(new_particles[index]));
        }
    }
}

RaoBlackwellSLAM::~RaoBlackwellSLAM()
{
    for (auto& map : maps)
        destroy_matrix(map);
}

static InitClass init("RaoBlackwellSLAM", &RaoBlackwellSLAM::Create, "Source/UserModules/EpiMove/RaoBlackwellSLAM/");


