/**
 * @file		main.cpp
 * @brief		Stand-alone FEdensity command line tool
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * Copyright 2017 Sigvald Marholm <marholm@marebakken.com>
 */

/* This file is part of FEdensity.
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
#include "FEdensity.h"
#include "polyhedron.h"
using std::cout;
using std::endl;

// Vertex v1 = {1,2,3};
// Vertex v2 = {2,3,4};
// Edge edge = {v1,v2};
// Face face = {edge,edge};

int main(){
    cout << "FEdensity " << VERSION << " running.\n";


    // Polyhedron p;
    // p.tetrahedron({{0,0,0},{1,0,0},{0,1,0},{0,0,1}});
    // // p.cube({0,0,0},{1,1,1});
    // p.clip({0.5,0,0},{0.5,0,0});
    // p.clip({0,0.5,0},{0,0.5,0});
    // p.clip({0,0,0.5},{0,0,0.5});
    // cout << p;
    // cout << p.volume() << "\n";

    Polyhedron p;
    p.cube({-1,-1,-1},{1,1,1});
    clock_t t = clock();
    for(int theta = 0; theta<360; theta++){
        double theta_rad = theta * 3.1415926535 / 180.0;
        double x = cos(theta_rad);
        double y = sin(theta_rad);
        p.clip({x,y,0},{x,y,0});
    }
    cout << "Cutting: " << 1000.0*(clock()-t)/CLOCKS_PER_SEC << "ms" << endl;
    t = clock();
    cout << p.volume() << "\n";
    cout << "Volume: " << 1000.0*(clock()-t)/CLOCKS_PER_SEC << "ms" << endl;
}
