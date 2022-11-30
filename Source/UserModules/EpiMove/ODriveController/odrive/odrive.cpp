//    Copyright (C) 2022 Daniel Calström Schad
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

#include "odrive.h"

namespace odrive {

ODrive::ODrive() = default;

ODrive::ODrive(libusb_context *context) {
    this->context = context;
}

ODrive::~ODrive() {
    if (handle) {
        libusb_release_interface(handle, 2);
        libusb_close(handle);
    };

    if (context) {
        libusb_exit(nullptr);
    }
}

ReturnStatus ODrive::search_device(const int vendor_id, const int product_id) {

    if (libusb_init(&context) != LIBUSB_SUCCESS) {
        fprintf(stdout, "Cannot initialize libusb\n");
        return STATUS_ERROR;
    }

    printf("Opening device %04X:%04X...\n", vendor_id, product_id);
    handle = libusb_open_device_with_vid_pid(context, vendor_id, product_id);

    if (handle == nullptr) {
        fprintf(stderr, "Unable to open device %04X:%04X\n", vendor_id, product_id);
        libusb_exit(context);
        return STATUS_ERROR;
    }

    if (libusb_kernel_driver_active(handle, 2) != LIBUSB_SUCCESS  &&
        libusb_detach_kernel_driver(handle, 2) != LIBUSB_SUCCESS) {
        fprintf(stderr, "Cannot acquire driver\n");
        libusb_close(handle);
        libusb_exit(context);
        return STATUS_ERROR;
    }

    if (libusb_claim_interface(handle, 2) != LIBUSB_SUCCESS) {
        fprintf(stderr, "Cannot claim interface\n");
        libusb_close(handle);
        libusb_exit(context);
        return STATUS_ERROR;
    }

    device = libusb_get_device(handle);

    if (device == nullptr) {
        fprintf(stderr, "Unable to get device\n");
        libusb_release_interface(handle, 2);
        libusb_close(handle);
        libusb_exit(context);
        return STATUS_ERROR;
    }

    libusb_device_descriptor descriptor{};
    libusb_get_device_descriptor(device, &descriptor);
    unsigned char serial_number[256];
    memset(serial_number, 0, sizeof(serial_number));

    if (libusb_get_string_descriptor_ascii(handle, descriptor.iSerialNumber, serial_number, sizeof(serial_number))
        < 0) { fprintf(stderr, "Failed to get serial number\n"); return STATUS_ERROR; }
    printf("Found ODrive with serial number: %s\n", serial_number);
    return STATUS_SUCCESS;
}

// The following contains, in a modified form, code written by Factor Robotics
// Copyright 2021 Factor Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

template <typename T>
ReturnStatus ODrive::read(short endpoint, T& value) {
    bytes request_payload;
    bytes response_payload;

    if (transaction(endpoint, sizeof(value), request_payload, response_payload, true) != STATUS_SUCCESS) {return STATUS_ERROR;}

    std::memcpy(&value, &response_payload[0], sizeof(value));
    return STATUS_SUCCESS;
}

template <typename T>
ReturnStatus ODrive::write(short endpoint, const T& value) {
    bytes request_payload;
    bytes response_payload;

    for (size_t i = 0; i < sizeof(value); ++i) {
        request_payload.emplace_back(((unsigned char*)&value)[i]);
    }

    return transaction(endpoint, 0, request_payload, response_payload, true);
}

ReturnStatus ODrive::call(short endpoint) {
    bytes request_payload;
    bytes response_payload;

    return transaction(endpoint, 0, request_payload, response_payload, true);
}

ReturnStatus ODrive::transaction(short endpoint, short response_size, const bytes& request_payload, bytes& response_payload, bool MSB) {

    if (MSB) {
        endpoint |= 0x8000;
    }
    sequence_number = (sequence_number + 1) & 0x7fff;
    sequence_number |= LIBUSB_ENDPOINT_IN;

    bytes response_packet;
    bytes request_packet = encode(sequence_number, endpoint, response_size, request_payload);

    int transferred = 0;
    if (libusb_bulk_transfer(handle, ODRIVE_OUT_ENDPOINT, request_packet.data(), request_packet.size(), &transferred, 0)
        != LIBUSB_SUCCESS) { return STATUS_ERROR; }

    if (MSB) {
        unsigned char response_data[ODRIVE_MAX_PACKET_SIZE] = { 0 };
        if (libusb_bulk_transfer(handle, ODRIVE_IN_ENDPOINT, response_data, ODRIVE_MAX_PACKET_SIZE, &transferred, 0)
            != LIBUSB_SUCCESS) { return STATUS_ERROR; }
        
        for (int i = 0; i < transferred; ++i) {
            response_packet.emplace_back(response_data[i]);
        }

        response_payload = decode(response_packet);
    }

    return STATUS_SUCCESS;
}

bytes ODrive::encode(short sequence_number, short endpoint, short response_size, const bytes& request_payload) {
    bytes packet;

    packet.emplace_back((sequence_number >> 0) & 0xFF);
    packet.emplace_back((sequence_number >> 8) & 0xFF);
    packet.emplace_back((endpoint >> 0) & 0xFF);
    packet.emplace_back((endpoint >> 8) & 0xFF);
    packet.emplace_back((response_size >> 0) & 0xFF);
    packet.emplace_back((response_size >> 8) & 0xFF);

    for (uint8_t b : request_payload)
    {
        packet.emplace_back(b);
    }

    short crc = ((endpoint & 0x7fff) == 0) ? ODRIVE_PROTOCOL_VERSION : odrive::endpoints::json_crc;
    packet.emplace_back((crc >> 0) & 0xFF);
    packet.emplace_back((crc >> 8) & 0xFF);

    return packet;
}

bytes ODrive::decode(const bytes& response_packet) {
    bytes payload;

    for (bytes::size_type i = 2; i < response_packet.size(); ++i) {
        payload.emplace_back(response_packet[i]);
    }
    return payload;
}

template ReturnStatus ODrive::read(short, bool&);
template ReturnStatus ODrive::read(short, float&);
template ReturnStatus ODrive::read(short, int32_t&);
template ReturnStatus ODrive::read(short, int64_t&);
template ReturnStatus ODrive::read(short, uint8_t&);
template ReturnStatus ODrive::read(short, uint16_t&);
template ReturnStatus ODrive::read(short, uint32_t&);
template ReturnStatus ODrive::read(short, uint64_t&);

template ReturnStatus ODrive::write(short, const bool&);
template ReturnStatus ODrive::write(short, const float&);
template ReturnStatus ODrive::write(short, const int32_t&);
template ReturnStatus ODrive::write(short, const int64_t&);
template ReturnStatus ODrive::write(short, const uint8_t&);
template ReturnStatus ODrive::write(short, const uint16_t&);
template ReturnStatus ODrive::write(short, const uint32_t&);
template ReturnStatus ODrive::write(short, const uint64_t&);
} // namespace odrive
