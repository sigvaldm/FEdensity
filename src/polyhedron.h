/**
 * @file		polyhedron.h
 * @brief		Polyhedron handling
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

#ifndef POLYHEDRON_H
#define POLYHEDRON_H

#include <vector>
#include <array>
using std::ostream;
using std::istream;
using std::vector;
using std::array;

/// Number of geometric dimensions.
constexpr int nDims = 3;

/// Geometric vectors (as opposed to stl::vector which is algebraic).
using Vector = array<double, nDims>;

///@name Geometric entities
///@{
using Vertex = Vector;              ///< A vertex.
using Edge   = array<Vertex, 2>;    ///< An edge/line given by its end points.
using Face   = vector<Edge>;        ///< A face consisting of several edges.
///@}

/**
 * @brief An arbitrarily shaped polyhedron in 3D-space.
 *
 * A polyhedron is really just a list of faces. However, we've added a few
 * additional methods for initialization, clipping and volume computation.
 *
 * The polyhedron is initially empty, and must be initialized using e.g.
 * tetrahedron() or cube().
 */
class Polyhedron : public vector<Face> {
public:
    /**
     * @brief Initialize tetrahedron.
     * @param   vertices    The four vertices of the tetrahedron.
     * @return              void
     */
    void tetrahedron(const vector<Vertex>& vertices);

    /**
     * @brief Initialize cube/box.
     * @param   lower   The vertex being the lowermost along all directions.
     * @param   upper   The vertex being the uppermost along all directions.
     * @return          void
     */
    void cube(const Vertex& lower, const Vertex& upper);

    /**
     * @brief Clip the polyhedron
     * @param   point   Some arbitrary point in the clipping plane.
     * @param   normal  A normal vector to the plane.
     * @param           void
     *
     * A plane is specified by its (not necessarily unit-length) normal vector
     * along with some point in the plane. The parts of the polyhedron in front
     * of the plane (where the normal vector points) are then trimmed away.
     */
    void clip(const Vector& point, const Vector& normal);

    /**
     * @brief Computes the volume
     * @return      The volume of the polyhedron.
     */
    double volume() const;
};

/**
 * @name Printing functions
 *
 * Overloading "put-to" operators for convenient printing.
 */
///@{
ostream& operator<<(ostream& out, const Vertex& vertex);
ostream& operator<<(ostream& out, const Edge& edge);
ostream& operator<<(ostream& out, const vector<Vertex>& edge);
ostream& operator<<(ostream& out, const Face& face);
ostream& operator<<(ostream& out, const Polyhedron& polyhedron);
///@}

#endif // POLYHEDRON_H
