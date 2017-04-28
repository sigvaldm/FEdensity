/**
 * @file		main.cpp
 * @brief		Stand-alone FEdensity command line tool
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 */

/*
 * Copyright 2017 Sigvald Marholm <marholm@marebakken.com>
 *
 * This file is part of FEdensity.
 *
 * FEdensity is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * FEdensity is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * FEdensity. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <cmath>
#include <ctime>
#include <numeric>
#include "FEdensity.h"
#include "polyhedron.h"
using std::cout;
using std::vector;
using fedensity::readGmsh;
using fedensity::writeVector;
using fedensity::Mesh;

int main(){
    cout << "FEdensity " << VERSION << " running.\n";


    Mesh mesh = readGmsh("mesh/regular1.msh");
    vector<double> volume = mesh.pittewayVolume();
    double totalVolume = std::accumulate(volume.begin(), volume.end(), 0.0f);
    cout << totalVolume << "\n";
    writeVector("volume.txt",volume);


    cout << "FEdensity " << VERSION << " ended successfully.\n";
    return 0;
}
