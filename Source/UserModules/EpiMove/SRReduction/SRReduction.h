//
//	SRReduction.h		This file is a part of the IKAROS project
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


#ifndef SRReduction_
#define SRReduction_

#include "IKAROS.h"

class SRReduction: public Module
{
public:
    static Module * Create(Parameter * p) { return new SRReduction(p); }

    SRReduction(Parameter * p) : Module(p) {}
    virtual ~SRReduction() {}

    void        SetSizes();
    void 		Init();
    void 		Tick();

    // Input arrays
    float ** sr1;
    float ** sr2;
    float ** sr3;
    float ** sr4;
    float ** sr5;
    float ** sr6;
    float ** sr7;
    float ** sr8;
    float ** sr9;
    float ** sr10;
    float ** sr11;

    // Output arrays

    float ** sr;
    int sr_size_x;
    int sr_size_y;

    // Helper arrays
    float ** denominator;
};

#endif

