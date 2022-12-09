//
//	ODriveController.cc		This file is a part of the IKAROS project
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

#include "TargetArrayControls.h"
#include <iostream>

// use the ikaros namespace to access the math library
// this is preferred to using <cmath>

using namespace ikaros;

void
TargetArrayControls::Init()
{
    // To get the parameters from the IKC file, use the Bind
    // function for each parameter. The parameters are initialized
    // from the IKC and can optionally be changed from the
    // user interface while Ikaros is running. If the parameter is not
    // set, the default value specified in the ikc-file will be used instead.

    io(pos_target, pos_target_size, "TARGET_ARRAY");
    set_array(pos_target, 0, pos_target_size);
}

void
TargetArrayControls::Tick()
{           

}

TargetArrayControls::~TargetArrayControls()
{

}

void TargetArrayControls::Command(std::string s, float x, float y, std::string value) {
    std::cout << "Got command: " << s << std::endl;
    if (s.find("left") != std::string::npos) left();
    if (s.find("right") != std::string::npos) right();
    if (s.find("forward") != std::string::npos) forward();
    if (s.find("backward") != std::string::npos) backward();
}

void TargetArrayControls::left() {
    pos_target[0] += -1;
    pos_target[1] += 1;
}

void TargetArrayControls::right() {
    pos_target[0] += 1;
    pos_target[1] += -1;
}

void TargetArrayControls::backward() {
    pos_target[0] += -1;
    pos_target[1] += -1;
}   

void TargetArrayControls::forward() {
    pos_target[0] += 1;
    pos_target[1] += 1;
}

// Install the module. This code is executed during start-up.

static InitClass init("TargetArrayControls", &TargetArrayControls::Create, "Source/UserModules/TargetArrayControls/");
