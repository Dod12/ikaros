#include "RaoBlackwellSLAM.h"

#include <algorithm>
#include <vector>

struct Pose
{
    float x;
    float y;
    float angle;
};

class Particle
{
    float ** grid;
    float grid_size_x;
    float grid_size_y;
    float grid_size;

    Pose pose;

    LidarSensor sensor;

    float weight;

    std::default_random_engine generator;

    float l_0;
    float l_occ;
    float l_free;

    float z_hit = 0.8f;
    float z_short = 0.1f;
    float z_max = 0.05f;
    float z_rand = 0.05f;

    float alpha = 0.1; // Velocity noise
    float beta = 0.1;  // Angular velocity noise

    float sigma_hit = 0.1; // Standard deviation of the Gaussian distribution for the hit model
    float lambda_short = 0.1; // Decay rate of the exponential distribution for the short model

    Particle(float ** grid, float grid_size_x, float grid_size_y, float grid_size, Pose pose, float weight, float prob_occ, float prob_free, float prob_unknown) :
        grid(grid), grid_size_x(grid_size_x), grid_size_y(grid_size_y), grid_size(grid_size), pose(pose), weight(weight) {
            generator = std::default_random_engine(std::random_device()());
            l_0 = log_odds(prob_unknown);
            l_occ = log_odds(prob_occ);
            l_free = log_odds(prob_free);
        }

    // Samples the motion model and returns the new pose
    Pose& sample_motion_model(float d_t, float velocity, float omega)
    {   
        std::normal_distribution distribution(0.0f, sqrt(alpha * velocity * velocity + beta * omega * omega));
        float v_hat = velocity + distribution(generator);
        float omega_hat = omega + distribution(generator);
        float gamma_hat = distribution(generator);

        pose.x = pose.x + (v_hat / omega_hat) * (sin(pose.angle + omega_hat * d_t) - sin(pose.angle));
        pose.y = pose.y + (v_hat / omega_hat) * (cos(pose.angle) - cos(pose.angle + omega_hat * d_t));
        pose.angle = pose.angle + omega_hat * d_t + gamma_hat * d_t;
        return pose;
    }

    // Calculates probability of current Lidar scan given the current pose and the old map
    float measurement_model(float * r_array, float * theta_array, int array_size)
    {
        float q = 1.0f;
        for (int i = 0; i < array_size; i++)
        {
            float z_star = ray_cast(theta_array[i]);
            float z = r_array[i];
            q *= z_hit * prob_hit(z_star, z) + z_short * prob_short(z_star, z) + z_max * prob_max(z_star, z) + z_rand * prob_rand(z_star, z);
        }
        return q;
    }

    // Ray cast on the map to find the "true" distanace to the obstacle in the direction of the Lidar beam
    float ray_cast(float angle)
    {
        float x = pose.x + sensor.x_offset;
        float y = pose.y + sensor.y_offset;
        float angle = pose.angle + sensor.angle_offset + angle;

        float x_step = grid_size * cos(angle);
        float y_step = grid_size * sin(angle);

        float dist = 0.0f;
        while (dist < sensor.max_range)
        {
            x += x_step;
            y += y_step;
            dist += grid_size;
            if (x < 0 || x >= grid_size_x || y < 0 || y >= grid_size_y)
                return sensor.max_range;
            if (grid[(int) x][(int) y] > log_odds(l_occ))
                return dist;
        }
        
    }

    float prob_hit(float z_star, float z)
    {
        // Normal distribution with mean z_star and standard deviation sigma_hit: return exp(-1.0f/2.0f * pow((z - z_star) / sigma_hit, 2))/sqrt(2 * M_PI * pow(sigma_hit, 2));
        // Normal distribution with mean z_star and standard deviation sigma_hit, normalized on the interval [0, sensor.max_range]
        return (sqrt(2 / M_PI) * exp(-pow(z - z_star, 2) / (2 * pow(sigma_hit, 2)))) / sigma_hit * (erf(z_star/sqrt(2) * sigma_hit) - erf(z_star - sensor.max_range / (sqrt(2) * sigma_hit)));
    }

    float prob_short(float z_star, float z)
    {
        float eta = 1 / (1 - exp(-lambda_short * z_star));
        return eta * lambda_short * exp(-lambda_short * z);
    }

    float prob_max(float z_star, float z)
    {
        if (z_star == sensor.max_range)
            return 1.0f;
        else
            return 0.0f;
    }

    float prob_rand(float z_star, float z)
    {
        return 1.0f / sensor.max_range;
    }

    // Iterate over scan beams and update grid cells
    void update_map(float * r_array, float * theta_array, int array_size)
    {
        for (int i = 0; i < array_size; ++i)
        {
            float x = pose.x + sensor.x_offset;
            float y = pose.y + sensor.y_offset;
            float angle = pose.angle + sensor.angle_offset + theta_array[i];

            float x_step = grid_size * cos(angle);
            float y_step = grid_size * sin(angle);

            float dist = 0.0f;
            while (dist < sensor.max_range)
            {
                x += x_step;
                y += y_step;
                dist += grid_size;
                if (x < 0 || x >= grid_size_x || y < 0 || y >= grid_size_y)
                    break;
                if (dist >= r_array[i])
                    grid[(int) x][(int) y] += l_occ - l_0;
                else if (dist < r_array[i])
                    grid[(int) x][(int) y] += l_free - l_0;
            }
        }   
    }
};