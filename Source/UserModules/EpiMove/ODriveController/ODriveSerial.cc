//
//    ODriveSerial.cc		This file is a part of the IKAROS project
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

#include "ODriveSerial.h"

namespace ODrive {

    template <typename T>
    Motors<T> make_motors(T left, T right)
    {
        return Motors<T>(left, right);
    }

    Feedback make_feedback(float position, float velocity)
    {
        return Feedback{position, velocity};
    }

    ODrive::ODriveSerial::ODriveSerial(const char * device_name, unsigned long baud_rate, unsigned long timeout)
            : Serial(device_name, baud_rate)
    {
        this->device_name = device_name;
        this->baud_rate = baud_rate;
        this->timeout = timeout;
    }

    ODrive::ODriveSerial::ODriveSerial(const std::string& device_name, unsigned long baud_rate, unsigned long timeout)
            : Serial(device_name.c_str(), baud_rate)
    {
        this->device_name = device_name.c_str();
        this->baud_rate = baud_rate;
        this->timeout = timeout;
    }

    ReturnStatus ODriveSerial::SendCommand(const std::string& command)
    {   
        std::string command_with_newline = command;
        if (command[command.length() - 1] != '\n')
        {
            command_with_newline += '\n';
        }
        const char *command_cstr = command_with_newline.c_str();
        int bytes_written = SendString(command_cstr);
        if (bytes_written == 0)
        {
            std::cerr << "Error writing command to ODrive" << std::endl;
            return ReturnStatus::ERROR;
        }
        else
        {
            return ReturnStatus::OK;
        }
    }

    std::string ODriveSerial::ReadString()
    {
        char * response = new char[MAX_RESPONSE_LENGTH];
        char delimiter = '\n';
        int response_length = ReceiveUntil(response, MAX_RESPONSE_LENGTH, delimiter, timeout);
        if (response_length == 0)
        {   
            std::cerr << "Error reading response from ODrive" << std::endl;
            return "";
        }
        else
        {
            return std::string(response);
        }
    }

    std::string ODriveSerial::ReadString(const std::string& command)
    {
        SendCommand(command);
        return ReadString();
    }

    std::vector<std::string> ODriveSerial::Tokenize(std::string str, const std::string& delimiter)
    {
        std::vector<std::string> tokens;
        size_t pos = 0;
        std::string token;
        while ((pos = str.find(delimiter)) != std::string::npos)
        {
            token = str.substr(0, pos);
            tokens.push_back(token);
            str.erase(0, pos + delimiter.length());
        }
        tokens.push_back(str);
        return tokens;
    }

    int ODriveSerial::ReadInt()
    {
        try {
            return std::stoi(ReadString());
        }
        catch (std::invalid_argument& e)
        {
            std::cerr << "Error reading int from ODrive" << std::endl;
            return 0;
        }
    }

    int ODriveSerial::ReadInt(const std::string& command)
    {
        try
        {
            return std::stoi(ReadString(command));
        }
        catch(std::invalid_argument& e)
        {
            std::cerr << "Error reading int from ODrive" << std::endl;
            return 0;
        }
        
    }

    float ODriveSerial::ReadFloat()
    {
        try
        {
            return std::stof(ReadString());
        }
        catch(std::invalid_argument& e)
        {
            std::cerr << "Error reading float from ODrive" << std::endl;
            return 0;
        }
    }

    float ODriveSerial::ReadFloat(const std::string& command)
    {
        try
        {
            return std::stof(ReadString(command));
        }
        catch(std::invalid_argument& e)
        {
            std::cerr << "Error reading float from ODrive" << std::endl;
            return 0;
        }
    }

    ReturnStatus ODriveSerial::SetAxisState(int axis, int state, int timeout, bool wait_for_idle) 
    {
        std::string command = "axis" + std::to_string(axis) + ".requested_state = " + std::to_string(state);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            if (wait_for_idle)
            {   
                int axis_state = 0;
                int counter = 0;
                do
                {   
                    usleep(50);
                    axis_state = ReadInt("axis" + std::to_string(axis) + ".current_state");
                    counter++;
                    if (counter > timeout/50)
                    {
                        std::cerr << "Timeout waiting for axis to go idle" << std::endl;
                        return ReturnStatus::ERROR;
                    }
                } while (axis_state != ODrive::AXIS_STATE_IDLE);
            }
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis state" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetState(int state, int timeout, bool wait_for_idle)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisState(axis, state, timeout, wait_for_idle);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        if (wait_for_idle)
        {
            int axis0_state = 0;
            int axis1_state = 0;
            int counter = 0;
            do
            {
                usleep(50);
                axis0_state = ReadInt("r axis0.current_state"); 
                axis1_state = ReadInt("r axis1.current_state");

                counter++;
                if (counter > timeout/50)
                {
                    std::cerr << "Timeout waiting for ODrive to go idle" << std::endl;
                    return ReturnStatus::ERROR;
                }
            } while (axis0_state != AXIS_STATE_IDLE && axis1_state != AXIS_STATE_IDLE);
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisPosition(int axis, float position)
    {
        std::string command = "p " + std::to_string(axis) + " " + std::to_string(position);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis position" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetPosition(std::array<float, 2> position)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisPosition(axis, position[axis]);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisPosition(int axis, float position, float velocity)
    {
        std::string command = "p " + std::to_string(axis) + " " + std::to_string(position) + " " + std::to_string(velocity);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis position" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetPosition(std::array<float, 2> position, std::array<float, 2> velocity)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisPosition(axis, position[axis], velocity[axis]);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisPosition(int axis, float position, float velocity, float current)
    {
        std::string command = "p " + std::to_string(axis) + " " + std::to_string(position) + " " + std::to_string(velocity) + " " + std::to_string(current);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis position" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetPosition(std::array<float, 2> position, std::array<float, 2> velocity, std::array<float, 2> current)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisPosition(axis, position[axis], velocity[axis], current[axis]);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisVelocity(int axis, float velocity)
    {
        std::string command = "v " + std::to_string(axis) + " " + std::to_string(velocity);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis velocity" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetVelocity(std::array<float, 2> velocity)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisVelocity(axis, velocity[axis]);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisVelocity(int axis, float velocity, float current)
    {
        std::string command = "v " + std::to_string(axis) + " " + std::to_string(velocity) + " " + std::to_string(current);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis velocity" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetVelocity(std::array<float, 2> velocity, std::array<float, 2> current)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisVelocity(axis, velocity[axis], current[axis]);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisCurrent(int axis, float current)
    {
        std::string command = "c " + std::to_string(axis) + " " + std::to_string(current);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis current" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetCurrent(std::array<float, 2> current)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisCurrent(axis, current[axis]);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::UpdateAxisWatchdog(int axis)
    {
        std::string command = "u " + std::to_string(axis);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error updating axis watchdog" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::UpdateWatchdog()
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = UpdateAxisWatchdog(axis);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisParameter(int axis, const std::string& parameter, float value)
    {
        std::string command = "w " + std::to_string(axis) + " " + parameter + " " + std::to_string(value);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis parameter" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetParameter(const std::string& parameter, float value)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisParameter(axis, parameter, value);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    ReturnStatus ODriveSerial::SetAxisParameter(int axis, const std::string& parameter, int value)
    {
        std::string command = "w " + std::to_string(axis) + " " + parameter + " " + std::to_string(value);
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error setting axis parameter" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::SetParameter(const std::string& parameter, int value)
    {
        for (auto axis : {0, 1})
        {
            ReturnStatus status = SetAxisParameter(axis, parameter, value);
            if (status == ReturnStatus::ERROR)
            {
                return ReturnStatus::ERROR;
            }
        }
        return ReturnStatus::OK;
    }

    float ODriveSerial::GetAxisParameterFloat(int axis, const std::string& parameter)
    {
        std::string command = "r " + std::to_string(axis) + " " + parameter;
        return ReadFloat(command);
    }

    Motors<float> ODriveSerial::GetParameterFloat(const std::string& parameter)
    {
        return make_motors(GetAxisParameterFloat(0, parameter),
                              GetAxisParameterFloat(1, parameter));
    }

    int ODriveSerial::GetAxisParameterInt(int axis, const std::string& parameter)
    {
        std::string command = "r " + std::to_string(axis) + " " + parameter;
        return ReadInt(command);
    }

    Motors<int> ODriveSerial::GetParameterInt(const std::string& parameter)
    {
        return make_motors(GetAxisParameterInt(0, parameter),
                              GetAxisParameterInt(1, parameter));
    }

    Feedback ODriveSerial::GetAxisFeedback(int axis)
    {
        std::string command = "f " + std::to_string(axis);
        std::string response = ReadString(command);
        std::vector<std::string> tokens = Tokenize(response, " ");
        if (tokens.size() != 2)
        {
            std::cerr << "Error getting axis feedback" << std::endl;
            return make_feedback(0.0f, 0.0f);
        }
        else
        {
            return make_feedback(std::stof(tokens[0]), std::stof(tokens[1]));
        }
    }

    Motors<Feedback> ODriveSerial::GetFeedback()
    {   
        return make_motors(GetAxisFeedback(0), GetAxisFeedback(1));
    }

    float ODriveSerial::GetAxisPos(int axis)
    {
        return GetAxisFeedback(axis).position;
    }

    Motors<float> ODriveSerial::GetPos()
    {
        return make_motors(GetAxisPos(0), GetAxisPos(1));
    }

    float ODriveSerial::GetAxisVel(int axis)
    {
        return GetAxisFeedback(axis).velocity;
    }

    Motors<float> ODriveSerial::GetVel()
    {
        return make_motors(GetAxisVel(0), GetAxisVel(1));
    }

    ReturnStatus ODriveSerial::SaveConfiguration()
    {
        std::string command = "ss";
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error saving configuration" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::EraseConfiguration()
    {
        std::string command = "se";
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error erasing configuration" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::Reboot()
    {
        std::string command = "sr";
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error rebooting" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

    ReturnStatus ODriveSerial::ClearErrors()
    {
        std::string command = "sc";
        ReturnStatus status = SendCommand(command);
        if (status == ReturnStatus::OK)
        {
            return ReturnStatus::OK;
        }
        else
        {
            std::cerr << "Error clearing errors" << std::endl;
            return ReturnStatus::ERROR;
        }
    }

} // namespace ODrive