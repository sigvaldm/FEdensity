/**
 * @file		main.cpp
 * @brief		Stand-alone GirafFE command line tool
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 */

/*
 * Copyright 2017 Sigvald Marholm <marholm@marebakken.com>
 *
 * This file is part of GirafFE.
 *
 * GirafFE is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * GirafFE is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * GirafFE. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <ctime>
#include <numeric>
#include "GirafFE.h"
#include "polyhedron.h"
using std::cout;
using std::cerr;
using std::vector;
using std::string;
using giraffe::readGmsh;
using giraffe::writeVector;
using giraffe::Mesh;
using poly::Point;

#include <array>

int main(int argc, char *argv[]){

    cout << "GirafFE " << VERSION << " running.\n";

    if(argc != 3){
        cerr << "Wrong syntax. Example: \"./giraffe input.msh output.txt\"\n";
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

    mesh.computeInfluencers();

    vector<double> volume = mesh.volume3voro();
    double totalVolume = std::accumulate(volume.begin(), volume.end(), 0.0f);
    cout << "Total volume: " << totalVolume << "\n";

    // outfile << std::hexfloat;
    writeVector(outfile, volume);

    cout << "GirafFE " << VERSION << " ended successfully.\n";
    return 0;
}
