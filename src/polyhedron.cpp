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

constexpr double epsilon = 1e-10;

/******************************************************************************
 * LOCAL FUNCTION DECLARATIONS
 *****************************************************************************/

/**
 * @brief Get vertices belonging to a face
 * @param   face    Face.
 * @return          Vertices in face.
 */
vector<Vertex> faceVertices(const Face& face);

/**
 * @brief Find the other edge a vertex belongs to.
 * @param   face    Face.
 * @param   edge    Curret edge.
 * @param   vertex  Vertex.
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
Edge otherEdge(const Face& face, const Edge& edge, const Vertex& vertex);

/**
 * @brief Find the other vertex (end) of an edge.
 * @param   edge    Edge.
 * @param   vertex  Current vertex.
 * @return          Other vertex.
 *
 * NB: This function is unorthodox in the sense that it compares two vertices,
 * consisting of floating point numbers, using ==. This gives higher speed
 * than using a tolerance and is expected to work in this special case since
 * vertices being the same are not computed independently but computed once and
 * then copied.
 */
Vertex otherVertex(const Edge& edge, const Vertex& vertex);

/**
 * @brief The euclidean length of an edge
 * @param   edge    Edge
 * @return          The euclidean (2-norm) length
 */
double length(const Edge& edge);

/******************************************************************************
 * POLYHEDRON METHODS
 *****************************************************************************/

Polyhedron::Polyhedron(const array<Vertex, nDims+1>& vertices){

    vector<Edge> edges;
    edges.reserve(6);
    (*this).reserve(4);

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

Polyhedron::Polyhedron(const Vertex& lower, const Vertex& upper){

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

double Polyhedron::volume() const{

    if(size()==0) return 0;

    double volume = 0;
    Vertex a = *(this->begin()->begin()->begin()); //(*this)[0][0][0];

    for(const auto& face : (*this)){

        vector<Vertex> vertices = faceVertices(face);
        auto start = vertices.begin();
        auto stop  = vertices.end();
        if(std::find(start, stop, a) == stop || true){
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
            if(length(trueNewEdge)>epsilon){
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

    Edge edge = *(face.begin());
    Vertex vertex = edge[0];
    Vertex start = vertex;

    do {
        vertices.push_back(vertex);
        edge = otherEdge(face, edge, vertex);
        vertex = otherVertex(edge, vertex);
    } while(vertex != start);

    return vertices;
}

double length(const Edge& edge){
    Vertex v = edge[1]-edge[0];
    double l = 0;
    for(const auto& c : v) l += c*c;
    return l;
}

/******************************************************************************
 * ARITHMETIC OPERATIONS (LOCAL)
 *****************************************************************************/

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

/******************************************************************************
 * PRINTING OPERATORS (LOCAL)
 *****************************************************************************/

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
