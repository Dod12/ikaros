//
//	  ODriveSerial.h		This file is a part of the IKAROS project
//
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
//
//    See http://www.ikaros-project.org/ for more information.
//

#include "IKAROS.h"
#include <iostream>
#include <array>
#include <unistd.h>
#include "ODriveEnums.h"

#define DEFAULT_BAUD_RATE 115200
#define MAX_RESPONSE_LENGTH 256
#define COMMAND_TIMEOUT 50 // ms


namespace ODrive {
    enum ReturnStatus
    {
        OK = 0,
        ERROR = 1,
        TIMEOUT = 2
    };

    class ODriveSerial: public Serial
    {
    private:
        ODrive::ReturnStatus SendCommand(const std::string& command);
        std::string ReadString();
        std::string ReadString(const std::string& command);
        std::vector<std::string> Tokenize(std::string str, const std::string& delimiter);
        int ReadInt();
        int ReadInt(const std::string& command);
        float ReadFloat();
        float ReadFloat(const std::string& command);

        const char * device_name;
        unsigned long baud_rate;
        unsigned long timeout;

    public:
        using Serial::Serial;

        explicit ODriveSerial(const char * serial_device, unsigned long baud_rate = DEFAULT_BAUD_RATE, unsigned long timeout = COMMAND_TIMEOUT);
        explicit ODriveSerial(const std::string& serial_device, unsigned long baud_rate = DEFAULT_BAUD_RATE, unsigned long timeout = COMMAND_TIMEOUT);

        ODrive::ReturnStatus SetAxisState(int axis, int state, int timeout = 1000, bool wait_for_idle = false);
        ODrive::ReturnStatus SetState(int state, int timeout = 1000, bool wait_for_idle = false);

        ODrive::ReturnStatus SetAxisPosition(int axis, float position);
        ODrive::ReturnStatus SetAxisPosition(int axis, float position, float velocity);
        ODrive::ReturnStatus SetAxisPosition(int axis, float position, float velocity, float current);
        ODrive::ReturnStatus SetPosition(std::array<float, 2> position);
        ODrive::ReturnStatus SetPosition(std::array<float, 2> position, std::array<float, 2> velocity);
        ODrive::ReturnStatus SetPosition(std::array<float, 2> position, std::array<float, 2> velocity, std::array<float, 2> current);

        ODrive::ReturnStatus SetAxisVelocity(int axis, float velocity);
        ODrive::ReturnStatus SetAxisVelocity(int axis, float velocity, float current);
        ODrive::ReturnStatus SetVelocity(std::array<float, 2> velocity);
        ODrive::ReturnStatus SetVelocity(std::array<float, 2> velocity, std::array<float, 2> current);

        ODrive::ReturnStatus SetAxisCurrent(int axis, float current);
        ODrive::ReturnStatus SetCurrent(std::array<float, 2> current);

        ODrive::ReturnStatus UpdateAxisWatchdog(int axis);
        ODrive::ReturnStatus UpdateWatchdog();

        ODrive::ReturnStatus SetAxisParameter(int axis, const std::string& parameter, float value);
        ODrive::ReturnStatus SetAxisParameter(int axis, const std::string& parameter, int value);
        ODrive::ReturnStatus SetParameter(const std::string& parameter, float value);
        ODrive::ReturnStatus SetParameter(const std::string& parameter, int value);

        float GetAxisParameterFloat(int axis, const std::string& parameter);
        int GetAxisParameterInt(int axis, const std::string& parameter);
        std::pair<float, float> GetParameterFloat(const std::string& parameter);
        std::pair<int, int> GetParameterInt(const std::string& parameter);

        std::pair<float, float> GetAxisFeedback(int axis);
        std::pair<std::array<float, 2>, std::array<float, 2>> GetFeedback();

        float GetAxisPos(int axis);
        std::array<float, 2> GetPos();

        float GetAxisVel(int axis);
        std::array<float, 2> GetVel();

        ODrive::ReturnStatus SaveConfiguration();
        ODrive::ReturnStatus EraseConfiguration();
        ODrive::ReturnStatus Reboot();
        ODrive::ReturnStatus ClearErrors();
    };
} // namespace ODrive