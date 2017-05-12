/**
 * @file		polyhedron.cpp
 * @brief		Polyhedron hanlding
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * Handles representation, cutting and volume computations of arbitrary
 * polyhedrons.
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
#include <algorithm>
#include <cmath>
#include <cassert>
#include "polyhedron.h"

namespace poly {

using std::vector;
using std::array;

/******************************************************************************
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************/

/**
 * @brief Find the other edge a vertex belongs to.
 * @param   face    Polygon.
 * @param   edge    Curret edge.
 * @param   vertex  Point.
 * @return          Other edge.
 *
 * A vertex alwas connects two edges in the same face (it may connect more,
 * but they don't belong to the same face). Given a face, a vertex and an edge,
 * find the other edge.
 *
 * NB: This function is unorthodox in the sense that it compares two edges,
 * consisting of floating point vertices, using ==. This gives higher speed
 * than using a tolerance and is expected to work in this special case since
 * vertices being the same are not computed independently but computed once and
 * then copied.
 */
Line otherEdge(const Polygon& face, const Line& edge, const Point& vertex);

/**
 * @brief Find the other vertex (end) of an edge.
 * @param   edge    Line.
 * @param   vertex  Current vertex.
 * @return          Other vertex.
 *
 * NB: This function is unorthodox in the sense that it compares two vertices,
 * consisting of floating point numbers, using ==. This gives higher speed
 * than using a tolerance and is expected to work in this special case since
 * vertices being the same are not computed independently but computed once and
 * then copied.
 */
Point otherVertex(const Line& edge, const Point& vertex);

/******************************************************************************
 * POINT
 *****************************************************************************/

Point::Point(double x, double y, double z){
    (*this)[0] = x;
    (*this)[1] = y;
    (*this)[2] = z;
}

Point::Point(double x, double y){
    (*this)[0] = x;
    (*this)[1] = y;
    (*this)[2] = 0;
}

Point::Point(double x){
    (*this)[0] = x;
    (*this)[1] = 0;
    (*this)[2] = 0;
}

double Point::length() const{
    double l = 0;
    for(const auto& c : *this) l += c*c;
    return std::sqrt(l);
}

/******************************************************************************
 * LINE
 *****************************************************************************/

Line::Line(const Point& a, const Point& b){
    (*this)[0] = a;
    (*this)[1] = b;
}

double Line::length() const{
    const Line& line = *this;
    Point v = line[1]-line[0];
    return v.length();
}

/******************************************************************************
 * POLYGON
 *****************************************************************************/

Polygon::Polygon(const std::array<Point, 3>& vertices){

    (*this).reserve(3);

    for(auto it = vertices.begin(); it != vertices.end(); ++it){
        for(auto it2 = it + 1; it2 != vertices.end(); ++it2){
            Line edge(*it, *it2);
            this->push_back(edge);
        }
    }
}

double Polygon::area() const{

    if(size()<3) return 0;

    double area = 0;
    vector<Point> vertices = this->vertices();

    Point a = vertices[0];

    auto start = vertices.begin();
    auto stop  = vertices.end();
    for(auto it=start+1; it != stop-1; it++){
        Point b = *it;
        Point c = *(it+1);
        Point temp = cross(b-a, c-a);
        area += temp.length();
    }
    area *= 0.5;
    return area;
}

void Polygon::clip(const Point& point, const Point& normal){

    Polygon &p = (*this);
    vector<Point> newEdge;

    for(auto edge = p.begin(); edge != p.end();){

        Point v1 = (*edge)[0];
        Point v2 = (*edge)[1];

        double v1normal = dot(v1-point, normal);
        double v2normal = dot(v2-point, normal);

        if(v1normal > -epsilon && v2normal > -epsilon){
            // edge is in front of wall. Remove it.
            edge = p.erase(edge);

        } else if(v1normal <= -epsilon && v2normal <= -epsilon){
            // edge is behind wall. Keep it as-is.
            ++edge;

        } else if(v1normal > -epsilon && v1normal <= epsilon){
            // v1 is in plane. Add it as a new point.
            newEdge.push_back(v1);
            ++edge;

        } else if(v2normal > -epsilon && v2normal <= epsilon){
            // v2 is in plane. Add it as a new point.
            newEdge.push_back(v2);
            ++edge;

        } else {
            // edge intersects plane.


            // double edgeNormal = dot(v2-v1, normal);
            // double alpha = -v1normal / edgeNormal;
            // Point vNew = v1 + alpha*(v2-v1);

            v2-=v1;
            double edgeNormal = dot(v2, normal);
            double alpha = -v1normal / edgeNormal;

            v2 *= alpha;
            v2 += v1;
            Point& vNew = v2;

            if(v1normal > epsilon)
                (*edge)[0] = vNew;
            else
                (*edge)[1] = vNew;

            newEdge.push_back(vNew);

            ++edge;
        }

    }

    assert(newEdge.size() == 0 || newEdge.size() == 2);
    if(newEdge.size() == 2){
        Line trueNewEdge = {newEdge[0], newEdge[1]};
        if(trueNewEdge.length()>epsilon){
            p.push_back(trueNewEdge);
        }
    }
}

/******************************************************************************
 * POLYHEDRON
 *****************************************************************************/

Polyhedron::Polyhedron(const array<Point, nDims+1>& vertices){

    vector<Line> edges;
    edges.reserve(6);
    (*this).reserve(4);

    for(auto it = vertices.begin(); it != vertices.end(); ++it){
        for(auto it2 = it + 1; it2 != vertices.end(); ++it2){
            Line edge(*it, *it2);
            edges.push_back(edge);
        }
    }

    this->push_back(Polygon {edges[3], edges[4], edges[5]});
    this->push_back(Polygon {edges[1], edges[2], edges[5]});
    this->push_back(Polygon {edges[0], edges[2], edges[4]});
    this->push_back(Polygon {edges[0], edges[1], edges[3]});
}

Polyhedron::Polyhedron(const Point& lower, const Point& upper){

    // Agreed, this function needs rewriting, but it works for now.

    constexpr int nCorners = 8;
    constexpr int nFaces = 6;

    vector<Point> vertices;
    vertices.reserve(nCorners);

    // Get all combinations of lower and upper in binary ordering
    for(int i=0; i<nCorners; i++){
        Point v;
        int temp = i;
        for(int d=nDims-1; d>=0 ; d--){
            v[d] = (temp%2) ? upper[d] : lower[d];
            temp /= 2;
        }
        vertices.push_back(v);
    }

    Polyhedron &p = (*this);
    p.reserve(nFaces);

    int arr[] = {1,2,4};
    for(int d=0; d<nDims; d++){
        int i = (d+1)%3;
        int j = (d+2)%3;
        for(int ul=0; ul<2; ul++){
            Point aa = vertices[ul*arr[d]];
            Point bb = vertices[ul*arr[d]+arr[i]];
            Point cc = vertices[ul*arr[d]+arr[i]+arr[j]];
            Point dd = vertices[ul*arr[d]+arr[j]];
            Line ab(aa,bb);
            Line bc(bb,cc);
            Line cd(cc,dd);
            Line da(dd,aa);
            p.push_back(Polygon {ab, bc, cd, da});
        }
    }
}

double Polyhedron::volume() const{

    // if(size()<4) return 0;

    double volume = 0;
    Point a = *(this->begin()->begin()->begin()); //(*this)[0][0][0];

    for(const auto& face : (*this)){

        vector<Point> vertices = face.vertices();
        auto start = vertices.begin();
        auto stop  = vertices.end();
        // if(std::find(start, stop, a) == stop){// || true){
            Point b = vertices[0];
            for(auto it=start+1; it != stop-1; ++it){
                // Point c(*it);
                Point c = *it;
                Point d = *(it+1);
                Point temp = cross(b-d, c-d);
                volume += std::abs(dot(a-d, temp));
            }
        // }
    }
    volume /= 6.0;
    return volume;
}

void Polyhedron::clip(const Point& point, const Point& normal){

    Polyhedron &p = (*this);
    Polygon newFace;
    newFace.reserve(10);

    for(auto face = p.begin(); face != p.end();){

        vector<Point> newEdge;
        newEdge.reserve(2);

        for(auto edge = face->begin(); edge != face->end();){

            const Point& v1 = (*edge)[0];
            const Point& v2 = (*edge)[1];

            // double v1normal = dot(v1-point, normal);
            // double v2normal = dot(v2-point, normal);

            double temp = dot(point,normal);
            double v1normal = dot(v1, normal) - temp;
            double v2normal = dot(v2, normal) - temp;

            if(v1normal > -epsilon && v2normal > -epsilon){
                // edge is in front of wall. Remove it.
                edge = face->erase(edge);

            } else if(v1normal <= -epsilon && v2normal <= -epsilon){
                // edge is behind wall. Keep it as-is.
                ++edge;

            } else if(v1normal > -epsilon && v1normal <= epsilon){
                // v1 is in plane. Add it as a new point.
                newEdge.push_back(v1);
                ++edge;

            } else if(v2normal > -epsilon && v2normal <= epsilon){
                // v2 is in plane. Add it as a new point.
                newEdge.push_back(v2);
                ++edge;

            } else {
                // edge intersects plane.

                Point vDiff = v2-v1;
                double edgeNormal = dot(vDiff, normal);
                double alpha = -v1normal / edgeNormal;

                vDiff *= alpha;
                vDiff += v1;
                // Point vNew = v1 + alpha*(v2);
                Point& vNew = vDiff;

                if(v1normal > epsilon)
                    (*edge)[0] = vNew;
                else
                    (*edge)[1] = vNew;

                newEdge.push_back(vNew);

                ++edge;
            }

        }

        assert(newEdge.size() == 0 || newEdge.size() == 2);
        if(newEdge.size() == 2){
            Line trueNewEdge = {newEdge[0], newEdge[1]};
            if(trueNewEdge.length()>epsilon){
                face->push_back(trueNewEdge);
                newFace.push_back(trueNewEdge);
            }
        }

        if(face->size()==0){
            face = p.erase(face);
        } else {
            ++face;
        }
    }

    if(newFace.size()>2)
        p.push_back(newFace);
}

/******************************************************************************
 * LOCAL HELPER FUNCTIONS
 *****************************************************************************/

Line otherEdge(const Polygon& face, const Line& edge, const Point& vertex){

    for(const auto& otherEdge : face){
        if(otherEdge != edge){
            auto start = otherEdge.begin();
            auto stop  = otherEdge.end();
            if(std::find(start, stop, vertex) != stop){
                return otherEdge;
            }
        }
    }
    return edge;
}

Point otherVertex(const Line& edge, const Point& vertex){
    return (vertex==edge[0] ? edge[1] : edge[0]);
}

vector<Point> Polygon::vertices() const {

    const Polygon& face = *this;
    vector<Point> vertices;

    Line edge = *(face.begin());
    Point vertex = edge[0];
    Point start = vertex;

    do {
        vertices.push_back(vertex);
        edge = otherEdge(face, edge, vertex);
        vertex = otherVertex(edge, vertex);
    } while(vertex != start);

    return vertices;
}

/******************************************************************************
 * PRINTING OPERATORS
 *****************************************************************************/

std::ostream& operator<<(std::ostream& out, const Point& point){

    auto it = point.begin();
    out << "(" << *it;

    for(it++; it != point.end(); it++){
        out << "," << *it;
    }
    return out << ")";
}

std::ostream& operator<<(std::ostream& out, const Line& line){

    auto it = line.begin();
    out << *it;

    for(it++; it != line.end(); it++){
        out << " - " << *it;
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const std::vector<Point>& points){

    auto it = points.begin();
    out << *it;

    for(it++; it != points.end(); it++){
        out << ", " << *it;
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const Polygon& polygon){

    out << "Polygon: \n";

    for(auto const& line : polygon){
        out << "  " << line << "\n";
    }
    return out;
}

std::ostream& operator<<(std::ostream& out, const Polyhedron& polyhedron){
    out << "Polyhedron: \n\n";
    for(auto const& polygon : polyhedron){
        out << polygon << "\n";
    }

    return out;
}

} // namespace poly
