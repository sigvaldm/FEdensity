/**
 * @file		polyhedron.h
 * @brief		Polyhedron hanlding
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
 *
 * Handles representation, cutting and volume computations of arbitrary
 * polyhedrons.
 *
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
