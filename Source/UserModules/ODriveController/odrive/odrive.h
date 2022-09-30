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

#pragma once

#include <libusb.h>
#include <cstdio>
#include <vector>
#include <map>
#include <string>

#include "odrive_endpoints.h"
#include "odrive_enums.h"

#define ODRIVE_USB_VENDORID 0x1209
#define ODRIVE_USB_PRODUCTID 0x0d32

#define ODRIVE_OUT_ENDPOINT 0x03
#define ODRIVE_IN_ENDPOINT 0x83

#define ODRIVE_PROTOCOL_VERSION 1
#define ODRIVE_MAX_PACKET_SIZE 16

namespace odrive {

    typedef std::vector<uint8_t> bytes;
    
    enum ReturnStatus {
        STATUS_SUCCESS = 0,
        STATUS_ERROR = -1,
    };

    class ODrive {
    public:
        ODrive();
        explicit ODrive(libusb_context *context);
        ~ODrive();

        ReturnStatus search_device(int vendor_id = ODRIVE_USB_VENDORID, int product_id = ODRIVE_USB_PRODUCTID);

        // ReturnStatus configure_device(const ODriveConfig& config);

        template <typename T>
        ReturnStatus read(short endpoint, T& value);
        template <typename T>
        ReturnStatus write(short endpoint, const T& value);
        ReturnStatus call(short endpoint);

    private:
        libusb_context* context = nullptr;
        struct libusb_device_handle* handle = nullptr;
        libusb_device* device = nullptr;

        short sequence_number = 0;

        static bytes encode(short sequence_number, short endpoint, short response_size, const bytes& request_payload);
        static bytes decode(const bytes& response_packet);

        ReturnStatus transaction(short endpoint, short response_size, const bytes& request_payload, bytes& response_payload, bool MSB);
    };
}