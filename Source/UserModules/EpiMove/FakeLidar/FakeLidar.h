//
//	MinimalModule.h		This file is a part of the IKAROS project
// 						
//    Copyright (C) 2012 <Author Name>
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


#ifndef FakeLidar_
#define FakeLidar_

#include "IKAROS.h"

using namespace sl;

class FakeLidar: public Module
{
public:
    static Module * Create(Parameter * p) { return new FakeLidar(p); }

    FakeLidar(Parameter * p) : Module(p) {}
    ~FakeLidar();

    void 		Init();
    void 		Tick();

    // Output arrays

    float * r_array;
    int r_array_size;

    float * theta_array;
    int theta_array_size;

    // Parameters

    int num_samples;
};

#endif

