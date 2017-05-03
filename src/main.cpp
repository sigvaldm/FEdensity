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
#include <fstream>
#include <cmath>
#include <ctime>
#include <numeric>
#include "FEdensity.h"
#include "polyhedron.h"
using std::cout;
using std::cerr;
using std::vector;
using std::string;
using fedensity::readGmsh;
using fedensity::writeVector;
using fedensity::Mesh;
using poly::Point;

#include <array>

int main(int argc, char *argv[]){

    cout << "FEdensity " << VERSION << " running.\n";

    if(argc != 3){
        cerr << "Wrong syntax. Example: \"./fedensity input.msh output.txt\"\n";
        return 1;
    }

    string ifname = argv[1];
    string ofname = argv[2];
    string ifext = ifname.substr(ifname.find_last_of(".")+1);

    std::ifstream infile(ifname);
    std::ofstream outfile(ofname, std::ofstream::out);

    Mesh mesh;

    if(ifext=="msh")
        readGmsh(infile, mesh);
    else if(ifext=="fe")
        readFE(infile, mesh);
    else {
        cerr << "File extension " << ifext << " not supported\n";
        return 1;
    }

    cout << mesh.cells[0] << "\n";
    cout << mesh.cells[1] << "\n";
    cout << mesh.cells[2] << "\n";

    mesh.computeInfluencers();

    // vector<double> volume = mesh.pittewayVolume();
    vector<double> volume = mesh.volume();
    double totalVolume = std::accumulate(volume.begin(), volume.end(), 0.0f);
    cout << "Total volume: " << totalVolume << "\n";

    // outfile << std::hexfloat;
    writeVector(outfile, volume);



    cout << "FEdensity " << VERSION << " ended successfully.\n";
    return 0;
}
