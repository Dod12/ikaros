//
//	MinimalModule.cc		This file is a part of the IKAROS project
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


#include "LidarSensor.h"
#include <iostream>

using namespace ikaros;

void
LidarSensor::Init()
{

    // Get parameters
    Bind(serial_port, "serial_port");
    Bind(baud_rate, "baud_rate");

    // Set output parameters
    io(x_array, x_array_size, "X_POSITION");
    io(y_array, y_array_size, "Y_POSITION");
    io(grid_matrix, grid_matrix_size_x, grid_matrix_size_y, "GRID");

    ///  Create a communication channel instance
    channel = createSerialPortChannel(serial_port, baud_rate);

    driver = *createLidarDriver();
    auto res = driver->connect(*channel);
    if (SL_IS_OK(res)) {
        sl_lidar_response_device_info_t deviceInfo;
        res = driver->getDeviceInfo(deviceInfo);
        if (SL_IS_OK(res)) {
            printf("Model: %d, Firmware Version: %d.%d,  Hardware Version: %d\n",
                   deviceInfo.model,
                   deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
                   deviceInfo.hardware_version);
            LidarScanMode mode{};
            driver->startScan(false, true, 0, &mode);
        } else {
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    } else {
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
    std::cout << "Initialization complete" << std::endl;
}

void
LidarSensor::Tick()
{
    res = driver->grabScanDataHq(measurements, measurements_array_size,0);
    if (SL_IS_OK(res)) {
        res = driver->ascendScanData(measurements, measurements_array_size);
        if (SL_IS_OK(res)) {
            max_distance = 0;
            for (int i = 0; i < x_array_size; ++i) {
                angle = (float) measurements[i].angle_z_q14 * 90.f / (1 << 14);
                distance = (float) measurements[i].dist_mm_q2 / 1000.f / (1 << 2);

                x_array[i] = cos(angle * 2 * M_PI / 360) * distance;
                y_array[i] = sin(angle * 2 * M_PI / 360) * distance;
                if (abs(x_array[i]) > max_distance) {
                    max_distance = abs(x_array[i]);
                }
                if (abs(y_array[i]) > max_distance) {
                    max_distance = abs(y_array[i]);
                }
            }
            for (int i = 0; i < grid_matrix_size_x; ++i) {
                for (int j = 0; j < grid_matrix_size_y; ++j) {
                    grid_matrix[i][j] = 0;
                }
            }
            for (int i = 0; i < x_array_size; ++i) {
                if (-max_distance <= x_array[i] && x_array[i] <= max_distance && -max_distance <= y_array[i] && y_array[i] <= max_distance) {
                    x_index = std::clamp((int) ((grid_matrix_size_x / 2) * (x_array[i] / max_distance) + (grid_matrix_size_x / 2)), 0, grid_matrix_size_x -  1);
                    y_index = std::clamp((int) ((grid_matrix_size_y / 2) * (y_array[i] / max_distance) + (grid_matrix_size_y / 2)), 0, grid_matrix_size_x -  1);
                    grid_matrix[x_index][y_index] = 1;
                } else {
                    fprintf(stderr, "Illegal value in distance arrays: (%f, %f)\n", x_array[i], y_array[i]);
                }

            }
        } else {
            fprintf(stderr, "Failed to sort scan data\r\n");
        }
    } else {
        fprintf(stderr, "Error while retrieving scan data\r\n");
    }
}

LidarSensor::~LidarSensor()
{
    driver->stop();
    delete driver;
    delete *channel;
}

static InitClass init("LidarSensor", &LidarSensor::Create, "Source/UserModules/LidarSensor/");


