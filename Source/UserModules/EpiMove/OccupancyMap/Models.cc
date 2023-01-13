#include "OccupancyMap.h"
#include <array>
#include <random>


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
        acc += (float) random_int(-b2,b2);
    return a + 0.5 * acc;
}


// Motion model for the occupancy map.
double motion_model_velocity(RobotLocation x_t, double linear_speed, double rotational_speed, RobotLocation x_t_1, double d_t)
{
    static const float a = 0.1;
    double linear_speed2 = pow(linear_speed, 2);
    double rotational_speed2 = pow(rotational_speed, 2);
    double mu = 0.5 * ((x_t_1.x - x_t.x)*cos(x_t_1.angle) + (x_t_1.y - x_t.y)*sin(x_t_1.angle))/((x_t_1.y - x_t.y)*cos(x_t_1.angle) - (x_t_1.x - x_t.x)*sin(x_t_1.angle));
    double x_star = (x_t_1.x + x_t.x)/2 + mu*(x_t_1.y - x_t.y);
    double y_star = (x_t_1.y + x_t.y)/2 + mu*(x_t.x - x_t_1.x);
    double r_star = sqrt(pow(x_t.x - x_star, 2) + pow(x_t.y - y_star, 2));
    double d_theta = atan2(x_t.y - y_star, x_t.x - x_star) - atan2(x_t_1.y - y_star, x_t_1.x - x_star);
    double v_hat = r_star * d_theta / d_t;
    double omega_hat = d_theta / d_t;
    double gamma_hat = (x_t.angle-x_t_1.angle)/d_t - omega_hat;
    return prob_normal_distribution(linear_speed - v_hat, a*(linear_speed2 + rotational_speed2)) *
           prob_normal_distribution(rotational_speed - omega_hat, a*(linear_speed2 + rotational_speed2)) *
           prob_normal_distribution(gamma_hat, a*rotational_speed2);
}

double motion_model_velocity(RobotLocation x_t, double left_speed, double right_speed, double wheelbase, RobotLocation x_t_1, double d_t)
{
    double linear_speed = (left_speed + right_speed)/2;
    double rotational_speed = (right_speed - left_speed)/wheelbase;
    return motion_model_velocity(x_t, linear_speed, rotational_speed, x_t_1, d_t);
}

RobotLocation sample_motion_model_velocity(double linear_speed, double rotational_speed, RobotLocation x_t_1, float delta_t)
{
    static const float a = 0.1;
    float linear_speed2 = pow(linear_speed, 2);
    float rotational_speed2 = pow(rotational_speed, 2);
    float v_hat = linear_speed + sample_normal_distribution((float) 0.0, a*(linear_speed2 + rotational_speed2));
    float omega_hat = rotational_speed + sample_normal_distribution((float) 0.0, a*(linear_speed2 + rotational_speed2));
    float gamma_hat = sample_normal_distribution((float) 0.0, a*(linear_speed2+rotational_speed2));

    return RobotLocation{x_t_1.x - v_hat/omega_hat*sin(x_t_1.angle) + v_hat/omega_hat*sin(x_t_1.angle + omega_hat*delta_t),
                         x_t_1.y + v_hat/omega_hat*cos(x_t_1.angle) - v_hat/omega_hat*cos(x_t_1.angle + omega_hat*delta_t),
                         x_t_1.angle + omega_hat*delta_t + gamma_hat*delta_t};
}


// Measurement model for the occupancy map.
double measurement_model_likelihood(float * z_radius, float * z_angle, size_t z_size, RobotLocation x_t, LidarSensor sensor, Map map)
{   
    static const float z_hit = 0.8;
    static const float z_random = 0.1;
    static const float z_max = 0.1;
    static const float sigma_hit = 0.1;
    double q = 1;
    for (size_t i = 0; i < z_size; ++i)
    {
        if (z_radius[i] == 0 || z_radius[i] >= sensor.max_distance)
            continue;
        double z_x = x_t.x + sensor.x_offset + z_radius[i]*cos(x_t.angle + sensor.theta_offset + z_angle[i]);
        double z_y = x_t.y + sensor.y_offset + z_radius[i]*sin(x_t.angle + sensor.theta_offset + z_angle[i]);
        float distance = map.GetClosestOccupiedDistance(z_x, z_y);
        q *= z_hit * prob_normal_distribution(distance, sigma_hit) + z_random / z_max;
    }
    return q;
}