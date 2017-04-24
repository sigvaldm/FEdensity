/**
 * @file		polyhedron.h
 * @brief		Polyhedron hanlding
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * Handles representation, cutting and volume computations of arbitrary
 * polyhedrons.
 */
#ifndef POLYHEDRON_H
#define POLYHEDRON_H

#include <vector>
#include <array>
using std::ostream;
using std::istream;
using std::vector;
using std::array;

constexpr int nDims = 3;

using Vertex = array<double, nDims>;
using Edge   = array<Vertex, 2>;
using Face   = vector<Edge>;

class Polyhedron : public vector<Face> {
public:
    void tetrahedron(const vector<Vertex>& vertices);
    void cube(const Vertex& lower, const Vertex& upper);
    double volume() const;
private:
};

vector<Vertex> faceVertices(const Face& face);

ostream& operator<<(ostream& out, const Vertex& vertex);
ostream& operator<<(ostream& out, const Edge& edge);
ostream& operator<<(ostream& out, const vector<Vertex>& edge);
ostream& operator<<(ostream& out, const Face& face);
ostream& operator<<(ostream& out, const Polyhedron& polyhedron);

#endif // POLYHEDRON_H
