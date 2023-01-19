#include "RaoBlackwellSLAM.h"

#include <algorithm>
#include <vector>

Particle::Particle(float ** grid, float grid_size_x, float grid_size_y, float cell_size, Pose pose, float weight, LidarSensor sensor, float l_0, float l_occ, float l_free, float z_hit, float z_short, float z_max, float z_rand, VelocityNoise vel_noise, float sigma_hit, float lambda_short)
{
    this->grid = grid;
    this->grid_size_x = grid_size_x;
    this->grid_size_y = grid_size_y;
    this->cell_size = cell_size;
    this->pose = pose;
    this->weight = weight;
    this->sensor = sensor;
    this->l_0 = l_0;
    this->l_occ = l_occ;
    this->l_free = l_free;
    this->z_hit = z_hit;
    this->z_short = z_short;
    this->z_max = z_max;
    this->z_rand = z_rand;
    this->vel_noise = vel_noise;
    this->sigma_hit = sigma_hit;
    this->lambda_short = lambda_short;
}

Particle Particle::operator=(const Particle& other)
{
    this->grid = other.grid;
    this->grid_size_x = other.grid_size_x;
    this->grid_size_y = other.grid_size_y;
    this->cell_size = other.cell_size;
    this->pose = other.pose;
    this->weight = other.weight;
    this->sensor = other.sensor;
    this->l_0 = other.l_0;
    this->l_occ = other.l_occ;
    this->l_free = other.l_free;
    this->z_hit = other.z_hit;
    this->z_short = other.z_short;
    this->z_max = other.z_max;
    this->z_rand = other.z_rand;
    this->vel_noise = other.vel_noise;
    this->sigma_hit = other.sigma_hit;
    this->lambda_short = other.lambda_short;
    return *this;
}

// Samples the motion model and returns the new pose based on velocity
Pose& Particle::sample_motion_model_velocity(float d_t, float velocity, float omega, std::default_random_engine& generator)
{   
    std::normal_distribution vel_distribution(0.0f, vel_noise.alpha1 * velocity * velocity + vel_noise.alpha2 * omega * omega);
    std::normal_distribution omega_distribution(0.0f, vel_noise.alpha3 * velocity * velocity + vel_noise.alpha4 * omega * omega);
    std::normal_distribution gamma_distribution(0.0f, vel_noise.alpha5 * velocity * velocity + vel_noise.alpha6 * omega * omega);
    float v_hat = velocity + vel_distribution(generator);
    float omega_hat = omega + omega_distribution(generator);
    float gamma_hat = gamma_distribution(generator);

    float d_x = (v_hat / omega_hat) * (sin(pose.angle + omega_hat * d_t) - sin(pose.angle));
    float d_y = (v_hat / omega_hat) * (cos(pose.angle) - cos(pose.angle + omega_hat * d_t));
    float d_angle = omega_hat * d_t + gamma_hat * d_t;

    if (!isnan(d_x) && !isnan(d_y) && !isnan(d_angle))
    {
        pose.x += d_x;
        pose.y += d_y;
        pose.angle += d_angle;
    }
    return pose;
}

// Samples the motion model and returns the new pose based on distance travelled
Pose& Particle::sample_motion_model_odometry(float d_l, float d_r, float wheelbase, std::default_random_engine& generator)
{
    std::normal_distribution<double> l_distribution(0.0f, vel_noise.alpha1 * d_l * d_l + vel_noise.alpha2 * d_r * d_r);
    std::normal_distribution<double> r_distribution(0.0f, vel_noise.alpha3 * d_l * d_l + vel_noise.alpha4 * d_r * d_r);
    double d_l_hat = d_l + l_distribution(generator);
    double d_r_hat = d_r + r_distribution(generator);

    static const double eps = 1e-5; // Epsilon to avoid division by zero

    if (abs(d_l_hat) < eps && abs(d_r_hat) < eps) // If the robot is not moving
    {
        return pose;
    } 
    else if (abs(d_l_hat - d_r_hat) < eps) // If the robot is moving straight
    {
        float d_s = (d_l_hat + d_r_hat) / 2;
        pose.x += d_s * cos(pose.angle);
        pose.y += d_s * sin(pose.angle);
        return pose;
    } 
    else if (abs(d_l_hat + d_l_hat) < eps) // If the robot is rotating in place
    {
        float d_theta = (d_r_hat - d_l_hat) / wheelbase;
        pose.angle += d_theta;
        return pose;
    } 
    else // If the robot is moving in an arc
    {
        double r = (wheelbase * (d_l_hat + d_r_hat)) / (2 * (d_r_hat - d_l_hat));
        double d_theta = (d_r_hat - d_l_hat) / wheelbase;

        pose.x += r * (-sin(pose.angle) + sin(pose.angle + d_theta));
        pose.y += r * (cos(pose.angle) - cos(pose.angle + d_theta));
        pose.angle += d_theta;
        return pose;
    }
}

// Calculates probability of current Lidar scan given the current pose and the old map
float Particle::measurement_model(float * r_array, float * theta_array, int array_size)
{
    weight = 1.0f;
    for (int i = 0; i < array_size; i++)
    {
        float z_star = ray_cast(theta_array[i]);
        float z = r_array[i];
        weight *= z_hit * prob_hit(z_star, z) + z_short * prob_short(z_star, z) + z_max * prob_max(z_star, z) + z_rand * prob_rand(z_star, z);
    }
    return weight;
}

// Ray cast on the map to find the "true" distance to the obstacle in the direction of the Lidar beam
float Particle::ray_cast(float angle)
{
    float x = pose.x + sensor.x_offset * cos(pose.angle) - sensor.y_offset * sin(pose.angle);
    float y = pose.y + sensor.x_offset * sin(pose.angle) + sensor.y_offset * cos(pose.angle);
    angle = pose.angle + sensor.angle_offset - angle;

    float x_step = cell_size * cos(angle);
    float y_step = cell_size * sin(angle);

    float dist = 0.0f;
    while (dist < sensor.max_range)
    {
        x += x_step;
        y += y_step;

        int x_index = (int) (x / cell_size);
        int y_index = (int) (y / cell_size);

        dist += cell_size;
        if (x_index < 0 || x_index >= grid_size_x || y_index < 0 || y_index >= grid_size_y)
            return sensor.max_range;
        else if (grid[x_index][y_index] < l_occ)
            return dist;
    }
    return sensor.max_range;
}

float Particle::prob_hit(float z_star, float z)
{
    // Normal distribution with mean z_star and standard deviation sigma_hit: return exp(-1.0f/2.0f * pow((z - z_star) / sigma_hit, 2))/sqrt(2 * M_PI * pow(sigma_hit, 2));
    // Normal distribution with mean z_star and standard deviation sigma_hit, normalized on the interval [0, sensor.max_range]
    if (0 <= z_star && z_star <= sensor.max_range)
        return (sqrt(2 / M_PI) * exp(-pow(z - z_star, 2) / (2 * pow(sigma_hit, 2)))) / sigma_hit * (erf(z_star/sqrt(2) * sigma_hit) - erf(z_star - sensor.max_range / (sqrt(2) * sigma_hit)));
    else
        return 0.0f;
}

float Particle::prob_short(float z_star, float z)
{
    float eta = 1 / (1 - exp(-lambda_short * z_star));
    return eta * lambda_short * exp(-lambda_short * z);
}

float Particle::prob_max(float z_star, float z)
{
    if (z_star == sensor.max_range)
        return 1.0f;
    else
        return 0.0f;
}

float Particle::prob_rand(float z_star, float z)
{
    return 1.0f / sensor.max_range;
}

// Iterate over scan beams and update grid cells
void Particle::update_map_raycasting(float * r_array, float * theta_array, int array_size, float alpha)
{
    // Calculate the position of the Lidar sensor in the world frame
    float x = pose.x + sensor.x_offset * cos(pose.angle) - sensor.y_offset * sin(pose.angle);
    float y = pose.y + sensor.x_offset * sin(pose.angle) + sensor.y_offset * cos(pose.angle);

    for (int i = 0; i < array_size; ++i)
    {
        float angle = pose.angle + sensor.angle_offset - theta_array[i];

        float x_step = cell_size * cos(angle);
        float y_step = cell_size * sin(angle);

        float dist = 0.0f;
        while (dist < sensor.max_range)
        {
            x += x_step;
            y += y_step;
            int x_index = (int) (x / cell_size);
            int y_index = (int) (y / cell_size);

            dist += cell_size;
            if (x_index < 0 || x_index >= grid_size_x || y_index < 0 || y_index >= grid_size_y)
                break;
            if (dist < r_array[i] - alpha / 2.0f)
                grid[x_index][y_index] += l_free - l_0;
            else if (dist > r_array[i] + alpha / 2.0f)
                break;
            else
                grid[x_index][y_index] += l_occ - l_0;
        }
    }   
}

// Iterate over grid cells and calculate the probability of occupancy
void Particle::update_map_beam_model(float * r_array, float * theta_array, int array_size, float alpha, float beta)
{
    for (int i = 0; i < grid_size_x; ++i)
    {
        for (int j = 0; j < grid_size_y; ++j)
        {
            float x = i * cell_size;
            float y = j * cell_size;

            float r = sqrt(pow(x - pose.x - sensor.x_offset, 2) + pow(y - pose.y - sensor.y_offset, 2));
            float phi = atan2(y - pose.y, x - pose.x) - pose.angle - sensor.angle_offset;

            auto subtraction = [&phi = phi](float theta, float smallest) { return abs(phi - theta) < abs(phi - smallest);};
            int k = std::min_element(theta_array, theta_array + array_size, subtraction) - theta_array;

            if (r > std::min(sensor.max_range, r_array[k] + alpha/2.0f) || abs(phi-theta_array[k]) > beta/2.0f)
                continue;
            if (r < sensor.max_range && abs(r - r_array[k]) < alpha/2.0f)
                grid[i][j] += l_occ - l_0;
            if (r < r_array[k] - alpha/2.0f)
                grid[i][j] += l_free - l_0;
        }
    }
}