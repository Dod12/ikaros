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


#ifndef LidarSensor_
#define LidarSensor_

#include "IKAROS.h"

#include <rplidar.h>

#define N_LIDAR_SAMPLES 8192

using namespace sl;

class LidarSensor: public Module
{
public:
    static Module * Create(Parameter * p) { return new LidarSensor(p); }

    LidarSensor(Parameter * p) : Module(p) {}
    ~LidarSensor();

    void 		Init();
    void 		Tick();

    // Output arrays

    float * x_array;
    int x_array_size;

    float * y_array;
    int y_array_size;

    float ** grid_matrix;
    int grid_matrix_size_x;
    int grid_matrix_size_y;

    // Parameters

    int baud_rate;
    std::string serial_port = {};

private:
    float angle, distance, max_distance, x, y;
    int x_index, y_index;

    Result<IChannel*> channel = Result<IChannel*>(nullptr);
    ILidarDriver* driver;
    sl_result res;
    sl_lidar_response_measurement_node_hq_t measurements[N_LIDAR_SAMPLES];
    size_t measurements_array_size = N_LIDAR_SAMPLES;
};

#endif

