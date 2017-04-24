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
#include <algorithm>
#include <cmath>
#include <cassert>
#include "polyhedron.h"
using std::cout;

constexpr double epsilon = 1e-10;

/*
 * These two functions do something very unorthodox for the sake of performance:
 * They compare two vertices or edges represented by doubles using ==. This is
 * a rare exception where this actually works. It is of interest to test whether
 * two vertices/edges are the same (stored redundantly). They are not arrived
 * upon by different calculations, but by copying from the same calculation.
 */
Edge otherEdge(const Face& face, const Edge& edge, const Vertex& vertex);
Vertex otherVertex(const Edge& edge, const Vertex& vertex);

double dot(const Vector& a, const Vector& b);
Vector cross(const Vector& a, const Vector& b);
Vector operator-(const Vector& lhs, const Vector& rhs);
Vector operator+(const Vector& lhs, const Vector& rhs);
Vector operator*(double lhs, const Vector& rhs);

ostream& operator<<(ostream& out, const Vertex& vertex){

    auto it = vertex.begin();
    out << "(" << *it;

    for(it++; it != vertex.end(); it++){
        out << "," << *it;
    }
    return out << ")";
}

ostream& operator<<(ostream& out, const Edge& edge){

    auto it = edge.begin();
    out << *it;

    for(it++; it != edge.end(); it++){
        out << " - " << *it;
    }
    return out;
}

ostream& operator<<(ostream& out, const vector<Vertex>& edge){

    auto it = edge.begin();
    out << *it;

    for(it++; it != edge.end(); it++){
        out << ", " << *it;
    }
    return out;
}

ostream& operator<<(ostream& out, const Face& face){

    out << "Face: \n";

    for(auto const& edge : face){
        out << "  " << edge << "\n";
    }
    return out;
}

ostream& operator<<(ostream& out, const Polyhedron& polyhedron){
    out << "Polyhedron: \n\n";
    for(auto const& face : polyhedron){
        out << face << "\n";
    }

    return out;
}

void Polyhedron::tetrahedron(const vector<Vertex>& vertices){

    vector<Edge> edges;
    edges.reserve(6);

    for(auto it = vertices.begin(); it != vertices.end(); it++){
        for(auto it2 = it + 1; it2 != vertices.end(); it2++){
            Edge edge = {*it, *it2};
            edges.push_back(edge);
        }
    }

    this->push_back(Face {edges[3], edges[4], edges[5]});
    this->push_back(Face {edges[1], edges[2], edges[5]});
    this->push_back(Face {edges[0], edges[2], edges[4]});
    this->push_back(Face {edges[0], edges[1], edges[3]});
}

void Polyhedron::cube(const Vertex& lower, const Vertex& upper){

    // Agreed, this function needs rewriting, but it works for now.

    constexpr int nCorners = 8;
    constexpr int nFaces = 6;

    vector<Vertex> vertices;
    vertices.reserve(nCorners);

    // Get all combinations of lower and upper in binary ordering
    for(int i=0; i<nCorners; i++){
        Vertex v;
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
            Vertex aa = vertices[ul*arr[d]];
            Vertex bb = vertices[ul*arr[d]+arr[i]];
            Vertex cc = vertices[ul*arr[d]+arr[i]+arr[j]];
            Vertex dd = vertices[ul*arr[d]+arr[j]];
            Edge ab = {aa, bb};
            Edge bc = {bb, cc};
            Edge cd = {cc, dd};
            Edge da = {dd, aa};
            p.push_back(Face {ab, bc, cd, da});
        }
    }
}

double dot(const Vector& a, const Vector& b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

Vector cross(const Vector& a, const Vector& b){
    Vertex res;
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res;
}

Vector operator-(const Vector& lhs, const Vector& rhs){
    Vertex res;
    for(size_t i=0; i<lhs.size(); i++){
         res[i] = lhs[i] - rhs[i];
    }
    return res;
}

Vector operator+(const Vector& lhs, const Vector& rhs){
    Vertex res;
    for(size_t i=0; i<lhs.size(); i++){
         res[i] = lhs[i] + rhs[i];
    }
    return res;
}

Vector operator*(double lhs, const Vector& rhs){
    Vertex res;
    for(size_t i=0; i<rhs.size(); i++){
        res[i] = lhs * rhs[i];
    }
    return res;
}

double Polyhedron::volume() const{

    if(size()==0) return 0;

    double volume = 0;
    Vertex a = (*this)[0][0][0];

    for(auto face : *this){

        vector<Vertex> vertices = faceVertices(face);

        auto start = vertices.begin();
        auto stop  = vertices.end();
        if(std::find(start, stop, a) == stop){

            Vertex b = vertices[0];
            for(auto it=start+1; it != stop-1; it++){
                Vertex c = *it;
                Vertex d = *(it+1);
                Vertex temp = cross(b-d, c-d);
                volume += std::abs(dot(a-d, temp));
            }
        }
    }
    volume /= 6.0;
    return volume;
}

Edge otherEdge(const Face& face, const Edge& edge, const Vertex& vertex){

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

Vertex otherVertex(const Edge& edge, const Vertex& vertex){
    return (vertex==edge[0] ? edge[1] : edge[0]);
}

vector<Vertex> faceVertices(const Face& face){

    vector<Vertex> vertices;

    Edge edge = face[0];
    Vertex vertex = edge[0];
    Vertex start = vertex;

    do {
        vertices.push_back(vertex);
        edge = otherEdge(face, edge, vertex);
        vertex = otherVertex(edge, vertex);
    } while(vertex != start);

    return vertices;
}

void Polyhedron::clip(const Vector& point, const Vector& normal){

    Polyhedron &p = (*this);
    Face newFace;

    for(auto face = p.begin(); face != p.end();){

        vector<Vertex> newEdge;

        for(auto edge = face->begin(); edge != face->end();){

            Vertex v1 = (*edge)[0];
            Vertex v2 = (*edge)[1];

            double v1normal = dot(v1-point, normal);
            double v2normal = dot(v2-point, normal);

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

                double edgeNormal = dot(v2-v1, normal);
                double alpha = -v1normal / edgeNormal;
                Vertex vNew = v1 + alpha*(v2-v1);

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
            Edge trueNewEdge = {newEdge[0], newEdge[1]};
            face->push_back(trueNewEdge);
            newFace.push_back(trueNewEdge);
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


void updateEdge(Edge& edge, const Vector& point, const Vector& normal){

    // Vertex v1 = edge[0];
    // Vertex v2 = edge[1];
    //
    // double v1normal = dot(v1-point, normal);
    // double v2normal = dot(v2-point, normal);
    //
    // if(v1normal > -epsilon && v2normal > -epsilon){
    //     // TBD: somehow remove edge
    //     //      return no new point
    // }
    //
    // if(v1normal <= -epsilon && v2normal <= -epsilon){
    //     // leave edge as-is
    //     // TBD: return no new point
    // }
    //
    // if(v1normal > -epsilon && v1normal <= epsilon){
    //     // leave edge as-is
    //     return v1;
    // }
    //
    // if(v2normal > -epsilon && v2normal <= epsilon){
    //     // leave edge as-is
    //     return v2;
    // }
    //
    // double edgeNormal = dot(v2-v1, normal);
    // double alpha = -v1normal / edgeNormal;
    // Vector vNew = v1 + alpha*(v2-v1);
    //
    // if(v1normal > epsilon)
    //     edge[0] = vNew;
    // else
    //     edge[1] = vNew;
    //
    // return vNew;
}

/*
def update_edge(edge, p, n):
    v1 = edge[0]
    v2 = edge[1]

    # Normal component wrt plane
    v1_normal = np.dot(v1-p,n)
    v2_normal = np.dot(v2-p,n)

    if v1_normal > -epsilon and v2_normal > -epsilon:
        return None, None

    if v1_normal <= -epsilon and v2_normal <= -epsilon:
        return edge, None

    if v1_normal > -epsilon and v1_normal <= epsilon: # in plane
        return edge, v1

    if v2_normal > -epsilon and v2_normal <= epsilon: # in plane
        return edge, v2

    edge_normal = np.dot(v2-v1,n)
    alpha = -v1_normal / edge_normal
    v_new = v1 + alpha*(v2-v1)

    if v1_normal > epsilon:
        edge[0] = v_new
    else:
        edge[1] = v_new

    assert edge_length(edge)>epsilon

    return edge, v_new
*/
