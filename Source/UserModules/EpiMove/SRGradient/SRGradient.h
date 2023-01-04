//
//	SRGradient.h		This file is a part of the IKAROS project
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


#ifndef SRGradient_
#define SRGradient_

#include "IKAROS.h"

class SRGradient: public Module
{
public:
    static Module * Create(Parameter * p) { return new SRGradient(p); }

    SRGradient(Parameter * p) : Module(p) {}
    ~SRGradient();

    void        SetSizes();
    void 		Init();
    void 		Tick();
    void        Command(std::string s, float x, float y, std::string value);

    // Gamma discount factor
    float gamma = 0.95;

    // Input arrays

    float ** successor_representation;
    int successor_representation_size_x;
    int successor_representation_size_y;

    // Output arrays

    float ** sr_gradient;
    int sr_gradient_size_x;
    int sr_gradient_size_y;

    float * target_position;
    int target_position_size_x;
};

#endif

