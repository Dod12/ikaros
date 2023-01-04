//
//	SRGradient.cc		This file is a part of the IKAROS project
//
//    Copyright (C) 2023 Daniel Carlström Schad
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

#include "SRGradient.h"

using namespace ikaros;

void SRGradient::SetSizes()
{
    // Set output matrix sizes
    int sizex = GetInputSizeX("SUCCESSOR_REPRESENTATION");
    int sizey = GetInputSizeY("SUCCESSOR_REPRESENTATION");
    SetOutputSize("SR_GRADIENT", sqrt(sizex), sqrt(sizey));
    SetOutputSize("TARGET_POSITION", 2);
}

void
SRGradient::Init()
{
    // Get input arrays
    io(successor_representation, successor_representation_size_x, successor_representation_size_y, "SUCCESSOR_REPRESENTATION");

    // Set output matrix
    io(sr_gradient, sr_gradient_size_x, sr_gradient_size_y, "SR_GRADIENT");
    io(target_position, target_position_size_x, "TARGET_POSITION");

    // Set default target position
    target_position[0] = sr_gradient_size_x / 2;
    target_position[1] = sr_gradient_size_y / 2;
}

void
SRGradient::Tick()
{
    // Calculate gradient of successor representation
    for (int i = 0; i < sr_gradient_size_x; ++i) {
        for (int j = 0; j < sr_gradient_size_y; ++j) {
            sr_gradient[i][j] = successor_representation[i * sr_gradient_size_x + j][(int) (target_position[1] * sr_gradient_size_x + target_position[0])];
        }
    }
}

void SRGradient::Command(std::string s, float x, float y, std::string value)
{
    if (s == "set_position") {
        target_position[0] = x;
        target_position[1] = y;
    }
}

SRGradient::~SRGradient()
{

}

static InitClass init("SRGradient", &SRGradient::Create, "Source/UserModules/EpiMove/SRGradient/");


