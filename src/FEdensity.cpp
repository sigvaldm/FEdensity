/**
 * @file		FEdensity.cpp
 * @brief		FEdensity API
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

#include "FEdensity.h"

#include <iostream>
#include <string>

namespace fedensity {

using std::cout;
using std::array;
using std::vector;
using poly::Point;
using poly::nDims;

using std::string;

vector<double> Mesh::volume3() const{

    using IdSet = Cell::IdSet;

    vector<double> volume(vertices.size());

    for(const auto& c : cells){

        PointArray vs;
        for(int i=0; i<nDims+1; i++) vs[i] = vertices[c.vertices[i]];

        const IdSet& influencers = c.influencers;

        for(const auto& i : influencers){
            poly::Polyhedron p(vs);

            for(const auto& j : influencers){
                if(j!=i){
                    Point point = 0.5*(vertices[j]+vertices[i]);
                    Point normal = vertices[j]-vertices[i];
                    p.clip(point, normal);
                }
            }

            volume[i] += p.volume();
        }
    }

    return volume;
}

vector<double> Mesh::volume2() const{

    using IdSet = Cell::IdSet;

    vector<double> volume(vertices.size());

    for(const auto& c : cells){

        std::array<Point, 3> vs;
        for(int i=0; i<3; i++) vs[i] = vertices[c.vertices[i]];

        const IdSet& influencers = c.influencers;

        for(const auto& i : influencers){
            poly::Polygon p(vs);

            for(const auto& j : influencers){
                if(j!=i){
                    Point point = 0.5*(vertices[j]+vertices[i]);
                    Point normal = vertices[j]-vertices[i];

                    // TBD: Why are these necessary?
                    point[2] = 0;
                    normal[2] = 0;

                    p.clip(point, normal);
                }
            }

            volume[i] += p.area();
        }
    }

    return volume;
}

vector<double> Mesh::volume() const{
    return (dim==3) ? volume3() : volume2();
}

vector<double> Mesh::pittewayVolume3() const{

    vector<double> volume(vertices.size());

    for(const auto& cell : cells){

        PointArray vs;
        for(int i=0; i<nDims+1; i++) vs[i] = vertices[cell.vertices[i]];

        for(int i=0; i<nDims+1; i++){ // for all vertices in cell
            poly::Polyhedron p(vs);

            for(int j=0; j<nDims+1; j++){ // clip away other vertices
                if(j!=i){
                    Point point = 0.5*(vs[j]+vs[i]);
                    Point normal = vs[j]-vs[i];
                    p.clip(point,normal);
                }
            }

            volume[cell.vertices[i]] += p.volume();
        }
    }

    return volume;
}

vector<double> Mesh::pittewayVolume2() const{

    vector<double> volume(vertices.size());

    for(const auto& cell : cells){
        std::array<Point, 3> vs;
        for(int i=0; i<3; i++) vs[i] = vertices[cell.vertices[i]];

        for(int i=0; i<3; i++){ // for all vertices in cell
            poly::Polygon p(vs);

            for(int j=0; j<3; j++){ // clip away other vertices
                if(j!=i){
                    Point point = 0.5*(vs[j]+vs[i]);
                    Point normal = vs[j]-vs[i];

                    // TBD: Why are these necessary?
                    point[2] = 0;
                    normal[2] = 0;

                    p.clip(point,normal);
                }
            }

            volume[cell.vertices[i]] += p.area();
        }
    }

    return volume;
}

vector<double> Mesh::pittewayVolume() const{
    return (dim==3) ? pittewayVolume3() : pittewayVolume2();
}

void Mesh::computeInfluencers(){
    for(size_t i=0; i<cells.size(); ++i){
        Point center = cellCircumcenter(i);
        propagate(center, cells[i], i);
    }
}

void Mesh::propagate(const poly::Point& center, const Cell& first, int cellIndex){

    using IdArray = Cell::IdArray;
    using IdSet = Cell::IdSet;

    PointArray vertices = cell(cellIndex);
    PointArray normals = facetNormals(cellIndex);
    IdArray neighbors = cells[cellIndex].neighbors;
    IdSet& influencers = cells[cellIndex].influencers;

    for(size_t i=0; i<dim+1; ++i)
        influencers.insert(first.vertices[i]);

    for(size_t i=0; i<dim+1; ++i){

        Point normal = normals[i];
        Point point = vertices[(i+1)%(dim+1)];

        if(dot(center-point, normal)>poly::epsilon && neighbors[i]>=0)
            propagate(center, first, neighbors[i]);

    }
}

inline double det(  double a, double b,
                    double c, double d){

    return a*d-b*c;
}

inline double det(  double a, double b, double c,
                    double d, double e, double f,
                    double g, double h, double i){

    return a*det(e,f,h,i) - b*det(d,f,g,i) + c*det(d,e,g,h);
}

PointArray Mesh::facetNormals(int index) const{

    PointArray normals;
    PointArray vertices = cell(index);

    if(dim==2) vertices[3] = Point(0,0,1);

    for(size_t i=0; i<dim+1; ++i){

        // Indices of the other vertices
        size_t j=(i+1)%(nDims+1);
        size_t k=(i+2)%(nDims+1);
        size_t l=(i+3)%(nDims+1);

        Point a = vertices[j];
        Point b = vertices[k];
        Point c = vertices[l];
        Point ab = b - a;
        Point ac = c - a;

        Point normal = cross(ab, ac);

        // Make sure it's outwards-pointing
        normal = (1-2*( dot(normal, vertices[i]-a)>0 ))*normal;

        normals[i] = normal;

    }

    return normals;

}

PointArray Mesh::cell(int index) const{

    PointArray vs;
    for(size_t i=0; i<dim+1; ++i)
        vs[i] = vertices[cells[index].vertices[i]];

    return vs;
}

Point Mesh::cellCircumcenter(int index) const{

    PointArray vs = cell(index);
    if(dim==3){

        // TBD: Perhaps inline (and place in header) vector operations?

        Point n0 = vs[1]-vs[0];
        Point n1 = vs[2]-vs[0];
        Point n2 = vs[3]-vs[0];
        Point p0 = 0.5*(vs[1]+vs[0]);
        Point p1 = 0.5*(vs[2]+vs[0]);
        Point p2 = 0.5*(vs[3]+vs[0]);
        Point b(dot(n0,p0),dot(n1,p1),dot(n2,p2));

        double denominator = det(   n0[0], n0[1], n0[2],
                                    n1[0], n1[1], n1[2],
                                    n2[0], n2[1], n2[2]);

        double num0 = det(          b[0] , n0[1], n0[2],
                                    b[1] , n1[1], n1[2],
                                    b[2] , n2[1], n2[2]);

        double num1 = det(          n0[0],  b[0], n0[2],
                                    n1[0],  b[1], n1[2],
                                    n2[0],  b[2], n2[2]);

        double num2 = det(          n0[0], n0[1],  b[0],
                                    n1[0], n1[1],  b[1],
                                    n2[0], n2[1],  b[2]);

        Point center;
        center[0] = num0/denominator;
        center[1] = num1/denominator;
        center[2] = num2/denominator;
        return center;

    } else { // dim==2

        Point n0 = vs[1]-vs[0];
        Point n1 = vs[2]-vs[0];
        Point p0 = 0.5*(vs[1]+vs[0]);
        Point p1 = 0.5*(vs[2]+vs[0]);
        Point b(dot(n0,p0),dot(n1,p1));

        double denominator = det(   n0[0], n0[1],
                                    n1[0], n1[1]);

        double num0 = det(          b[0] , n0[1],
                                    b[1] , n1[1]);

        double num1 = det(          n0[0],  b[0],
                                    n1[0],  b[1]);

        Point center;
        center[0] = num0/denominator;
        center[1] = num1/denominator;
        center[2] = 0;
        return center;

    }
}

std::ostream& operator<<(std::ostream& out, const Cell& cell){
    out << "Vertices:";
    for(const auto& v : cell.vertices) out << v << " ";
    out << "\n";

    out << "Neighbors:";
    for(const auto& n : cell.neighbors) out << n << " ";
    out << "\n";

    out << "Influencing vertices:";
    for(const auto& i : cell.influencers) out << i << " ";
    out << "\n";

    return out;
}

} // namespace fedensity
