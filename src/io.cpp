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
using std::vector;
using poly::Point;

using std::cout;
using std::cerr;

namespace fedensity {

std::istream& readGmsh(std::istream& in, Mesh &mesh){

    // The resizes in this function has a very small (~10 ms) performance
    // penalty because STL containers are value-initialized. How to circumvent
    // this can found here: http://stackoverflow.com/a/21028912/273767

    string line;

    while(in >> line){
        if(line=="$Nodes") break;
    }

    int nNodes;
    in >> nNodes;
    mesh.vertices.resize(nNodes);

    int id;
    double x, y, z;
    for(int i=0; i<nNodes; i++){
        in >> id >> x >> y >> z;
        mesh.vertices[id-1] = Point(x,y,z);
    }

    while(in >> line){
        if(line=="$Elements") break;
    }

    int nCells;
    in >> nCells;
    mesh.cells.reserve(nCells);

    int type, nTags, garbage;
    int v1, v2, v3, v4;
    for(int i=0; i<nCells; i++){

        in >> id >> type >> nTags;
        for(int n=0; n<nTags; n++) in >> garbage;

        Cell cell;

        if(type==4){ // Tetrahedron
            in >> v1 >> v2 >> v3 >> v4;
            cell.vertices = {v1-1, v2-1, v3-1, v4-1};
            mesh.dim = 3;
        } else if(type==2){ // Triangle
            in >> v1 >> v2 >> v3;
            cell.vertices = {v1-1, v2-1, v3-1};
            mesh.dim = 2;
        } else {
            cerr << "Only supports triangular or tetrahedral elements.\n";
            exit(1);
        }

        mesh.cells.push_back(cell);

    }

    return in;
}

std::istream& readFE(std::istream& in, Mesh &mesh){

    // The resize in this function has a very small (~10 ms) performance
    // penalty because STL containers are value-initialized. How to circumvent
    // this can found here: http://stackoverflow.com/a/21028912/273767

    string line;

    while(in >> line) if(line=="Dimensions") break;
    int dim;
    in >> dim;
    mesh.dim = dim;

    while(in >> line) if(line=="Neighbors") break;
    bool neighbors;
    in >> neighbors;

    while(in >> line) if(line=="Nodes") break;
    int nNodes;
    in >> nNodes;
    mesh.vertices.resize(nNodes);

    int id;
    double x, y, z;
    for(int i=0; i<nNodes; i++){
        in >> id >> x >> y >> z;
        mesh.vertices[id-1] = Point(x,y,z);
    }

    while(in >> line) if(line=="Elements") break;
    int nCells;
    in >> nCells;
    mesh.cells.reserve(nCells);

    int v1, v2, v3, v4;
    for(int i=0; i<nCells; i++){

        Cell cell;

        if(dim==3){ // Tetrahedron
            in >> id >> v1 >> v2 >> v3 >> v4;
            cell.vertices = {v1-1, v2-1, v3-1, v4-1};
            if(neighbors){
                in >> v1 >> v2 >> v3 >> v4;
                cell.neighbors = {v1-1, v2-1, v3-1, v4-1};
            }
        } else if(dim==2){ // Triangle
            in >> id >> v1 >> v2 >> v3;
            cell.vertices = {v1-1, v2-1, v3-1};
            if(neighbors){
                in >> v1 >> v2 >> v3;
                cell.neighbors = {v1-1, v2-1, v3-1};
            }
        } else {
            cerr << "Only supports triangular or tetrahedral elements.\n";
            exit(1);
        }

        mesh.cells.push_back(cell);

    }

    return in;
}

std::ostream& writeVector(std::ostream& out, const vector<double>& vec){
    for(const auto& elem : vec){
        out << elem << "\n";
    }
    return out;
}

} // namespace fedensity
