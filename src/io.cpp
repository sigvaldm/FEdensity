/**
 * @file		io.cpp
 * @brief		Reading from and writing to files
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
#include <sstream>
#include <ios>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>
#include "GirafFE.h"
using std::ifstream;
using std::ofstream;
using std::getline;
using std::string;
using std::istringstream;
using std::vector;
using poly::Point;

using std::cout;
using std::cerr;

namespace giraffe {

std::istream& readGmsh(std::istream& in, Mesh &mesh){

    // The resizes in this function has a very small (~10 ms) performance
    // penalty because STL containers are value-initialized. How to circumvent
    // this can found here: http://stackoverflow.com/a/21028912/273767

    string line;
    std::istringstream ins;

    while(getline(in, line)) if(line=="$Nodes") break;
    int nNodes;
    getline(in, line);
    ins = std::istringstream(line);
    ins >> nNodes;
    mesh.vertices.resize(nNodes);

    int id;
    double x, y, z;
    for(int i=0; i<nNodes; i++){
        getline(in, line);
        ins = std::istringstream(line);
        ins >> id >> x >> y >> z;
        // in >> id >> x >> y >> z;
        mesh.vertices[id-1] = Point(x,y,z);
    }

    while(getline(in, line)) if(line=="$Elements") break;
    int nCells;
    getline(in, line);
    ins = std::istringstream(line);
    ins >> nCells;
    mesh.cells.reserve(nCells);

    vector<vector<int>> belongsTo;
    belongsTo.resize(nNodes);

    int dim = 2; // changes to 3 upon detection of tetrahedron

    int type, nTags, garbage;
    int v1, v2, v3, v4;
    // int ctr=0;
    for(int i=0; i<nCells; ++i){

        getline(in, line);
        ins = std::istringstream(line);

        ins >> id >> type >> nTags;
        // assert(++ctr==id);
        for(int n=0; n<nTags; ++n) ins >> garbage;

        if(dim==2 && type==4){ // turns out to be 3D
            dim = 3;

            // if any triangles are fetched they must be thrown away
            mesh.cells.clear();
            belongsTo.clear();
            belongsTo.resize(nNodes);
        }

        if(type==4 && dim==3){ // Tetrahedron
            ins >> v1 >> v2 >> v3 >> v4;
            Cell cell;
            cell.vertices = {v1-1, v2-1, v3-1, v4-1};
            mesh.cells.push_back(cell);
            int index = mesh.cells.size()-1;
            belongsTo[v1-1].push_back(index);
            belongsTo[v2-1].push_back(index);
            belongsTo[v3-1].push_back(index);
            belongsTo[v4-1].push_back(index);
        } else if(type==2 && dim==2){ // Triangle
            ins >> v1 >> v2 >> v3;
            Cell cell;
            cell.vertices = {v1-1, v2-1, v3-1};
            mesh.cells.push_back(cell);
            int index = mesh.cells.size()-1;
            belongsTo[v1-1].push_back(index);
            belongsTo[v2-1].push_back(index);
            belongsTo[v3-1].push_back(index);
        }

    }

    mesh.dim = dim;
    mesh.findNeighbors(belongsTo);

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

    vector<vector<int>> belongsTo;
    if(!neighbors) belongsTo.resize(nNodes);

    int v1, v2, v3, v4;
    for(int i=0; i<nCells; i++){

        Cell cell;

        if(dim==3){ // Tetrahedron
            in >> id >> v1 >> v2 >> v3 >> v4;
            cell.vertices = {v1-1, v2-1, v3-1, v4-1};
            if(neighbors){
                in >> v1 >> v2 >> v3 >> v4;
                cell.neighbors = {v1-1, v2-1, v3-1, v4-1};
            } else {
                belongsTo[v1-1].push_back(i);
                belongsTo[v2-1].push_back(i);
                belongsTo[v3-1].push_back(i);
                belongsTo[v4-1].push_back(i);
            }
        } else if(dim==2){ // Triangle
            in >> id >> v1 >> v2 >> v3;
            cell.vertices = {v1-1, v2-1, v3-1};
            if(neighbors){
                in >> v1 >> v2 >> v3;
                cell.neighbors = {v1-1, v2-1, v3-1};
            } else {
                belongsTo[v1-1].push_back(i);
                belongsTo[v2-1].push_back(i);
                belongsTo[v3-1].push_back(i);
            }
        } else {
            cerr << "Only supports triangular or tetrahedral elements.\n";
            exit(1);
        }

        mesh.cells.push_back(cell);

    }

    if(!neighbors) mesh.findNeighbors(belongsTo);

    return in;
}

void Mesh::findNeighbors(const std::vector<std::vector<int>>& belongsTo){

    int nCells = cells.size();
    for(int c=0; c<nCells; ++c){
        Cell& cell = cells[c];

        for(size_t i=0; i<dim+1; ++i){

            // facet opposite of vertex i
            int j = (i+1)%(dim+1);
            int k = (i+2)%(dim+1);

            int vj = cell.vertices[j];
            int vk = cell.vertices[k];

            vector<int> candidates = belongsTo[vj];

            // Remove self
            auto it = std::find(candidates.begin(), candidates.end(), c);
            candidates.erase(it);

            // Remove candidates not containing vk
            for(auto it = candidates.begin(); it != candidates.end();){
                const Cell& candidate = cells[*it];

                bool found = false;
                for(size_t d=0; d<dim+1; ++d)
                    if(candidate.vertices[d] == vk) found = true;

                if(found) ++it;
                else it = candidates.erase(it);
            }

            if(dim==3){
                int l = (i+3)%(dim+1);
                int vl = cell.vertices[l];

                // Remove candidates not containing vl
                for(auto it = candidates.begin(); it != candidates.end();){
                    const Cell& candidate = cells[*it];

                    bool found = false;
                    for(size_t d=0; d<dim+1; ++d)
                        if(candidate.vertices[d] == vl) found = true;

                    if(found) ++it;
                    else it = candidates.erase(it);
                }
            }

            assert(candidates.size()<=1);

            if(candidates.size()==1)
                cell.neighbors[i] = candidates[0];
            else
                cell.neighbors[i] = -1;

        }

    }
}

std::ostream& writeVector(std::ostream& out, const vector<double>& vec){
    for(const auto& elem : vec){
        out << elem << "\n";
    }
    return out;
}

} // namespace giraffe
