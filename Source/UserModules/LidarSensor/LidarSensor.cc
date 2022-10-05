//
//	MinimalModule.cc		This file is a part of the IKAROS project
//
//    Copyright (C) 2022 Daniel Carlström Schad
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
    io(r_array, r_array_size, "R_ARRAY");
    io(theta_array, theta_array_size, "THETA_ARRAY");

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
    std::this_thread::sleep_for(std::chrono::seconds(10));
    res = driver->grabScanDataHq(measurements, measurements_array_size, 0);
    if (SL_IS_FAIL(res)) throw;
    std::thread poller_thread(&LidarSensor::poller, this, 100);
    poller_thread.detach();
}

void
LidarSensor::Tick()
{

}

void LidarSensor::poller(int sleep_millis)
{
    while (get_poller()) {
        res = driver->grabScanDataHq(measurements, measurements_array_size, 0);
        if (SL_IS_OK(res)) {
            if (r_array_size != theta_array_size) { Notify(msg_fatal_error, "R_ARRAY and THETA_ARRAY must be of same size"); }
            
            for (int i = 0; i < r_array_size; ++i) {
                theta_array[i] = ((float) measurements[i].angle_z_q14 * 90.f / (1 << 14)) * 2 * M_PI / 360;
                r_array[i] = (float) measurements[i].dist_mm_q2 / 1000.f / (1 << 2);
            }
        } else {
            fprintf(stderr, "Error while retrieving scan data: %i\r\n", res);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_millis));
    }
}

void LidarSensor::stop_poller() { poller_mutex.lock(); run_poller = false; poller_mutex.unlock(); }

bool LidarSensor::get_poller() { bool ret; poller_mutex.lock(); ret = run_poller; poller_mutex.unlock(); return ret; }

LidarSensor::~LidarSensor()
{
    driver->stop();
    delete driver;
    delete *channel;
}

static InitClass init("LidarSensor", &LidarSensor::Create, "Source/UserModules/LidarSensor/");


