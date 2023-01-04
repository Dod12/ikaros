//
//	SRReduction.cc		This file is a part of the IKAROS project
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


#include "SRReduction.h"

using namespace ikaros;

void
SRReduction::SetSizes()
{
    sr_size_x = GetInputSizeX("SR1");
    sr_size_y = GetInputSizeY("SR1");
    SetOutputSize("MEAN_SR", sr_size_x, sr_size_y);
}

void
SRReduction::Init()
{
    io(sr1, sr_size_x, sr_size_y, "SR1");
    io(sr2, sr_size_x, sr_size_y, "SR2");
    io(sr3, sr_size_x, sr_size_y, "SR3");
    io(sr4, sr_size_x, sr_size_y, "SR4");
    io(sr5, sr_size_x, sr_size_y, "SR5");
    io(sr6, sr_size_x, sr_size_y, "SR6");
    io(sr7, sr_size_x, sr_size_y, "SR7");
    io(sr8, sr_size_x, sr_size_y, "SR8");
    io(sr9, sr_size_x, sr_size_y, "SR9");
    io(sr10, sr_size_x, sr_size_y, "SR10");
    io(sr11, sr_size_x, sr_size_y, "SR11");

    io(sr, sr_size_x, sr_size_y, "MEAN_SR");

    denominator = create_matrix(sr_size_x, sr_size_y);
    set_matrix(denominator, 11, sr_size_x, sr_size_y);
}



void
SRReduction::Tick()
{
    set_matrix(sr, 0, sr_size_x, sr_size_y);
    for (auto matrix : {sr1, sr2, sr3, sr4, sr5, sr6, sr7, sr8, sr9, sr10, sr11})
    {
        add(sr, matrix, sr_size_x, sr_size_y);
    }
    divide(sr, sr, denominator, sr_size_x, sr_size_y);
}



static InitClass init("SRReduction", &SRReduction::Create, "Source/UserModules/EpiMove/SRReduction/");


