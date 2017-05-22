/**
 * @file		GirafFE.cpp
 * @brief		GirafFE API
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

#include "GirafFE.h"
#include "voro++.hh"

#include <iostream>
#include <string>


namespace gfe {

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

vector<double> Mesh::volume3voro() const{

    using IdSet = Cell::IdSet;

    size_t maxInfluencers = 0;

    vector<double> volume(vertices.size());

    for(const auto& c : cells){

        PointArray vs;
        for(int i=0; i<nDims+1; i++) vs[i] = vertices[c.vertices[i]];

        const IdSet& influencers = c.influencers;

        maxInfluencers = std::max(maxInfluencers, influencers.size());


        for(const auto& i : influencers){

            voro::voronoicell p;
            const Point &a = vs[0];
            const Point &b = vs[1];
            const Point &c = vs[2];
            const Point &d = vs[3];
            p.init_tetrahedron(a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2], d[0], d[1], d[2]);

            for(const auto& j : influencers){
                if(j!=i){
                    Point n = vertices[j]-vertices[i];
                    Point po = 0.5*(vertices[j]+vertices[i]);
                    double nlen = n.length();
                    n /= nlen;
                    double d = dot(po,n);
                    p.nplane(n[0], n[1], n[2], 2*d, 0);
                }
            }

            double pvol = p.volume();
            assert(pvol>=0.0);

            volume[i] += pvol;
        }
    }

    cout << "Max influencers: " << maxInfluencers << "\n";

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

GraphNode<size_t> Mesh::computeInfluencersStatistics(){
    GraphNode<size_t> root(-1);
    for(size_t i=0; i<cells.size(); ++i){
        Point center = cellCircumcenter(i);
        propagateStatistics(center, cells[i], i, root);
    }
    return root;
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

void Mesh::propagateStatistics(const poly::Point& center, const Cell& first, int cellIndex, GraphNode<size_t>& parent){

    using IdArray = Cell::IdArray;
    using IdSet = Cell::IdSet;

    parent.children.push_back(GraphNode<size_t>(cellIndex));
    GraphNode<size_t> &thisNode = *(parent.children.end()-1);

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
            propagateStatistics(center, first, neighbors[i], thisNode);

    }
}

inline void argmax2of4(std::array<double, 4> v, int& a, int& b){

    // Optimal sorting network

    std::array<int, 4> i = {0,1,2,3};

    if(v[i[0]] > v[i[1]]) std::swap(i[0], i[1]);
    if(v[i[2]] > v[i[3]]) std::swap(i[2], i[3]);
    if(v[i[0]] > v[i[2]]) std::swap(i[0], i[2]);
    if(v[i[1]] > v[i[3]]) std::swap(i[1], i[3]);
    if(v[i[1]] > v[i[2]]) std::swap(i[1], i[2]);

    a = i[3];
    b = i[2];
}

inline void argmax2of4alt(std::array<double, 4> v, int& a, int& b){

    // TBD: Untested

    // Better than optimal sorting network by not sorting completely.
    //
    // Uses the following home-made network:
    // _________
    // _|___|___
    // ___|___|_
    // _______|_
    //
    // "Bubbles" out the minimum of three and three wires.
    //
    // The upper branches will contain the two maximal in arbitrary order.

    std::array<int, 4> i = {0,1,2,3};

    if(v[i[2]] > v[i[3]]) std::swap(i[2], i[3]);
    if(v[i[1]] > v[i[2]]) std::swap(i[1], i[2]);
    if(v[i[2]] > v[i[3]]) std::swap(i[2], i[3]);
    if(v[i[0]] > v[i[2]]) std::swap(i[0], i[2]);

    a = i[3];
    b = i[2];
}

inline void argmax1of3(std::array<double, 4> v, int& a){
    if(v[0] < v[1])
        if(v[1] < v[2]) a = 2;
        else            a = 1;
    else                a = 0;
}

void Mesh::propagateRestricted(const poly::Point& center, const Cell& first, int cellIndex){

    using IdArray = Cell::IdArray;
    using IdSet = Cell::IdSet;

    PointArray vertices = cell(cellIndex);
    PointArray normals = facetNormals(cellIndex);
    IdArray neighbors = cells[cellIndex].neighbors;
    IdSet& influencers = cells[cellIndex].influencers;

    for(size_t i=0; i<dim+1; ++i)
        influencers.insert(first.vertices[i]);

    std::array<double, nDims+1> centerNormals;

    for(size_t i=0; i<dim+1; ++i){

        Point normal = normals[i];
        Point point = vertices[(i+1)%(dim+1)];

        centerNormals[i] = dot(center-point, normal);

    }

    if(dim==2){
        int i;
        argmax1of3(centerNormals, i);

        if(centerNormals[i]>poly::epsilon && neighbors[i]>=0)
            propagateRestricted(center, first, neighbors[i]);

    } else { // dim==3
        int i, j;
        argmax2of4(centerNormals, i, j);

        if(centerNormals[i]>poly::epsilon && neighbors[i]>=0)
            propagateRestricted(center, first, neighbors[i]);

        if(centerNormals[j]>poly::epsilon && neighbors[j]>=0)
            propagateRestricted(center, first, neighbors[j]);
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

} // namespace giraffe
