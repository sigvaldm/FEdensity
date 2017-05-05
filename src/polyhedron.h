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
#include <initializer_list>
#include <iostream>

#include <cassert>

namespace poly {

/// Maximum number of geometric dimensions.
constexpr int nDims = 3;

constexpr double epsilon = 1e-10;

/**
 * @brief A cartesian point or geometric (not algebraic) vector.
 *
 * 1D and 2D vectors can be represented by simply not using all coordinates.
 *
 * A fixed-size std::array is used instead of std::vector to eliminate a level
 * of pointer dereferring in the algorithms using this structure thereby
 * improving performance at the cost of always allocating 3 elements even for
 * 1D and 2D vectors.
 *
 * NB: Inheriting (especially publicly) from STL is a controversial topic. The
 * defense for doing so is that this _is_ an array with all its native features.
 * One of the often quoted reasons for not inheriting STL containers is their
 * lack of virtual destructors. This pose no problem here because no additional
 * member variables are allocated. Why not just use an alias? Because:
 *  1. We want to benefit from static type-checking.
 *  2. We like to add special operators/functions for this type only.
 *  3. For consistency with the Polyhedron type.
 */
class Point : public std::array<double, nDims> {
public:
    Point(){}                              ///< Uninitialized
    Point(double x);                       ///< 1D initialization
    Point(double x, double y);             ///< 2D initialization
    Point(double x, double y, double z);   ///< 3D initialization
    double length() const;                 ///< Euclidean length
    Point& operator-=(const Point& rhs);
    Point& operator+=(const Point& rhs);
    Point& operator*=(const Point& rhs);
    Point& operator*=(double rhs);
};

/******************************************************************************
 * ARITHMETIC OPERATIONS
 *****************************************************************************/

inline Point& Point::operator+=(const Point& rhs){
    Point &lhs = *this;
    for(size_t i=0; i<nDims; ++i) lhs[i] += rhs[i];
    return lhs;
}

inline Point& Point::operator-=(const Point& rhs){
    Point &lhs = *this;
    for(size_t i=0; i<nDims; ++i) lhs[i] -= rhs[i];
    return lhs;
}

inline Point& Point::operator*=(const Point& rhs){
    Point &lhs = *this;
    for(size_t i=0; i<nDims; ++i) lhs[i] *= rhs[i];
    return lhs;
}

inline Point& Point::operator*=(double rhs){
    Point &lhs = *this;
    for(size_t i=0; i<nDims; ++i) lhs[i] *= rhs;
    return lhs;
}

inline Point operator+(Point lhs, const Point& rhs){
    lhs += rhs;
    return lhs;
}

inline Point operator-(Point lhs, const Point& rhs){
    lhs -= rhs;
    return lhs;
}

inline Point operator*(Point lhs, double rhs){
    lhs *= rhs;
    return lhs;
}

inline Point operator*(double lhs, const Point& rhs){
    return rhs*lhs;
}

inline double dot(const Point& a, const Point& b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

// same as dot(vec-point, normal) but more efficient
// inline double normalComp(const Point& vec, const Point& point, const Point& normal){
//     return
// }

inline Point cross(const Point& a, const Point& b){
    Point res;
    res[0] = a[1]*b[2] - a[2]*b[1];
    res[1] = a[2]*b[0] - a[0]*b[2];
    res[2] = a[0]*b[1] - a[1]*b[0];
    return res;
}

/**
 * @brief A straight line represented by two end-points.
 *
 * A fixed-size std::array as opposed to vectors improves performance due to
 * one less level of pointer dereferring.
 *
 * NB: See Point for discussion of inherting from STL.
 */
class Line : public std::array<Point, 2> {
public:
    Line(){}                                ///< Uninitialized
    Line(const Point& a, const Point& b);   ///< Initialized by end-points
    double length() const;                  ///< Euclidean length
};

/**
 * @brief A polygon represented by a set of edges.
 *
 * NB: See Point for discussion of inherting from STL.
 */
class Polygon : public std::vector<Line> {
public:
    /// Uninitialized Polygon
    Polygon(){}

    /**
     * @brief Initialize triangle.
     * @param   vertices    The three vertices of the triangle.
     */
    Polygon(const std::array<Point, 3>& vertices);

    /// Polygon with specified lines
    Polygon(const std::initializer_list<Line>& v) : vector<Line>(v) {}

    /// Return vertices in polygon
    std::vector<Point> vertices() const;

    double area() const;

    void clip(const Point& point, const Point& normal);
};
///@}

/**
 * @brief A polyhedron represented by a set of faces.
 *
 * NB: See Point for discussion of inherting from STL.
 */
class Polyhedron : public std::vector<Polygon> {
public:

    /// Uninitialized Polyhedron
    Polyhedron(){}

    /**
     * @brief Initialize tetrahedron.
     * @param   vertices    The four vertices of the tetrahedron.
     */
    Polyhedron(const std::array<Point, nDims+1>& vertices);

    /**
     * @brief Initialize cube/box.
     * @param   lower   The vertex being the lowermost along all directions.
     * @param   upper   The vertex being the uppermost along all directions.
     */
    Polyhedron(const Point& lower, const Point& upper);

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
    void clip(const Point& point, const Point& normal);

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
std::ostream& operator<<(std::ostream& out, const Point& vertex);
std::ostream& operator<<(std::ostream& out, const Line& edge);
std::ostream& operator<<(std::ostream& out, const std::vector<Point>& edge);
std::ostream& operator<<(std::ostream& out, const Polygon& face);
std::ostream& operator<<(std::ostream& out, const Polyhedron& polyhedron);
///@}

} // namespace poly

#endif // POLYHEDRON_H
