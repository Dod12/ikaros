//
//	RoutePlanner.h		This file is a part of the IKAROS project
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


#ifndef RoutePlanner_
#define RoutePlanner_

#include "IKAROS.h"

class RoutePlanner: public Module
{
public:
    static Module * Create(Parameter * p) { return new RoutePlanner(p); }

    RoutePlanner(Parameter * p) : Module(p) {}
    ~RoutePlanner();

    void        SetSizes();
    void 		Init();
    void 		Tick();
    void        Command(std::string s, float x, float y, std::string value);

    // Robot position
    int start_pos_x;
    int start_pos_y;

    // Input arrays

    float ** sr_gradient;
    int sr_gradient_size_x;
    int sr_gradient_size_y;

    float * target_position;
    int target_position_size_x;

    float * start_position;
    int start_position_size_x;

    // Output arrays

    float ** route;
    int route_size_x;
    int route_size_y;
};

#endif

