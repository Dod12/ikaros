//
//	SuccessorRepresentation.h		This file is a part of the IKAROS project
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


#ifndef SuccessorRepresentation_
#define SuccessorRepresentation_

#include "IKAROS.h"

class SuccessorRepresentation: public Module
{
public:
    static Module * Create(Parameter * p) { return new SuccessorRepresentation(p); }

    SuccessorRepresentation(Parameter * p) : Module(p) {}
    ~SuccessorRepresentation();

    void        SetSizes();
    void 		Init();
    void 		Tick();
    void        Command(std::string s, float x, float y, std::string value);

    // Gamma discount factor
    float gamma = 0.95;

    // Input arrays

    float ** transition_matrix;
    int transition_matrix_size_x;
    int transition_matrix_size_y;

    // Output arrays

    float ** successor_representation;
    int successor_representation_size_x;
    int successor_representation_size_y;

    std::pair<int, int> sr_gradient_position = {0, 0};
    float ** sr_gradient;
    int sr_gradient_size_x;
    int sr_gradient_size_y;

    // Helper array
    float ** identity;
    float ** intermediate;

};

#endif

