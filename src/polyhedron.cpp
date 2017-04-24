/**
 * @file		polyhedron.cpp
 * @brief		Polyhedron hanlding
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * Handles representation, cutting and volume computations of arbitrary
 * polyhedrons.
 */

#include <iostream>
#include <algorithm>
#include <cmath>
#include "polyhedron.h"
using std::cout;

/*
 * These two functions do something very unorthodox for the sake of performance:
 * They compare two vertices or edges represented by doubles using ==. This is
 * a rare exception where this actually works. It is of interest to test whether
 * two vertices/edges are the same (stored redundantly). They are not arrived
 * upon by different calculations, but by copying from the same calculation.
 */
Edge otherEdge(const Face& face, const Edge& edge, const Vertex& vertex);
Vertex otherVertex(const Edge& edge, const Vertex& vertex);
double dot(const Vertex& a, const Vertex& b);
Vertex cross(const Vertex& a, const Vertex& b);

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

double dot(const Vertex& a, const Vertex& b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

Vertex cross(const Vertex& a, const Vertex& b){
    Vertex res;
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res;
}

Vertex operator-(const Vertex& lhs, const Vertex& rhs){
    Vertex res;
    for(size_t i=0; i<lhs.size(); i++){
         res[i] = lhs[i] - rhs[i];
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
/*
def volume(self):
    if len(self.faces)==0: return 0.0

    volume = 0.0
    a = self.faces[0][0][0]
    for face in self.faces:
        face_vertices = extract_face_vertices(face)
        if not vertex_in_list(a,face_vertices):
            b = face_vertices[0]
            for i in range(1,len(face_vertices)-1):
                c = face_vertices[i]
                d = face_vertices[i+1]
                cross = np.cross(b-d,c-d)
                dot = np.dot(a-d,cross)
                volume += abs(dot)
    volume *= (1.0/6)
    return volume
*/

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
