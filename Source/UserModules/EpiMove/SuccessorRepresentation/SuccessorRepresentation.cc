//
//	SuccessorRepresentation.cc		This file is a part of the IKAROS project
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

#include "SuccessorRepresentation.h"

using namespace ikaros;

void SuccessorRepresentation::SetSizes()
{
    // Set output matrix sizes
    int sizex = GetInputSizeX("TRANSITION_MATRIX");
    int sizey = GetInputSizeY("TRANSITION_MATRIX");
    SetOutputSize("SUCCESSOR_REPRESENTATION", sizex, sizey);
}

void
SuccessorRepresentation::Init()
{

    // Set parameters
    Bind(gamma, "gamma");

    // Get input arrays
    io(transition_matrix, transition_matrix_size_x, transition_matrix_size_y, "TRANSITION_MATRIX");

    // Set output matrix
    io(sucessor_representation, sucessor_representation_size_x, sucessor_representation_size_y, "SUCCESSOR_REPRESENTATION");

    identity = create_matrix(transition_matrix_size_x, transition_matrix_size_y);
    identity = eye(identity, transition_matrix_size_x);

    intermediate = create_matrix(transition_matrix_size_x, transition_matrix_size_y);
}

void
SuccessorRepresentation::Tick()
{
    // Calculate sucessor representation = 1/(I - gamma * T)
    multiply(intermediate, transition_matrix, gamma, transition_matrix_size_x, transition_matrix_size_y);

    subtract(intermediate, identity, intermediate, transition_matrix_size_x, transition_matrix_size_y);

    pinv(sucessor_representation, intermediate, transition_matrix_size_x, transition_matrix_size_y);
}

SuccessorRepresentation::~SuccessorRepresentation()
{
    destroy_matrix(identity);
    destroy_matrix(intermediate);
}

static InitClass init("SuccessorRepresentation", &SuccessorRepresentation::Create, "Source/UserModules/EpiMove/SuccessorRepresentation/");


