/**
 * @file		main.cpp
 * @brief		Stand-alone FEdensity command line tool
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 */

#include <iostream>
#include <cmath>
#include "FEdensity.h"
#include "polyhedron.h"
using std::cout;

Vertex v1 = {1,2,3};
Vertex v2 = {2,3,4};
Edge edge = {v1,v2};
Face face = {edge,edge};

int main(){
    cout << "FEdensity " << VERSION << " running.\n";

    Polyhedron p;
    // p.tetrahedron({{0,0,0},{1,0,0},{0,1,0},{0,0,1}});
    p.cube({0,0,0},{1,1,1});
    cout << p;
    // cout << faceVertices(p[0]) << "\n";
    cout << p.volume() << "\n";

    // auto vertices = faceVertices(p[1]);
    // cout << vertices << "\n";

    return 0;
}
