/**
 * @file		io.cpp
 * @brief		Reading from and writing to files
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
#include <ios>
#include <fstream>
#include <string>
#include <cassert>
#include "FEdensity.h"
using std::ifstream;
using std::ofstream;
using std::getline;
using std::string;
using std::istringstream;

using std::cout;

namespace fedensity {

Mesh readGmsh(const string& filename){

    Mesh mesh;
    ifstream infile(filename);

    string line;
    while(infile >> line){
        if(line=="$Nodes") break;
    }

    int nNodes;
    infile >> nNodes;
    mesh.vertices.reserve(nNodes);

    int id, id_ = 1;
    double x, y, z;
    for(int i=0; i<nNodes; i++){
        infile >> id >> x >> y >> z;
        assert(id==id_++);
        mesh.vertices.push_back(Point(x,y,z));
    }

    infile >> line;
    assert(line=="$EndNodes");

    while(infile >> line){
        if(line=="$Elements") break;
    }

    int nCells;
    infile >> nCells;
    mesh.cells.reserve(nCells);

    id_ = 1;
    int type, nTags, a, b;
    int v1, v2, v3, v4;
    for(int i=0; i<nCells; i++){
        infile >> id >> type >> nTags >> a >> b >> v1 >> v2 >> v3 >> v4;
        assert(id==id_++);
        assert(type==4); // tetrahedron
        assert(nTags==2);

        Cell cell;
        cell.vertices = {v1-1, v2-1, v3-1, v4-1};
        mesh.cells.push_back(cell);
    }

    infile >> line;
    assert(line=="$EndElements");

    return mesh;

}

void writeVector(const string& filename, const vector<double>& vec){

    ofstream outfile(filename);
    // outfile << std::hexfloat;
    for(const auto& elem : vec){
        outfile << elem << "\n";
    }
}

} // namespace fedensity
