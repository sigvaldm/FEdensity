/**
 * @file		GirafFE.h
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

 #ifndef GIRAFFE_H
 #define GIRAFFE_H

#include <vector>
#include <array>
#include <utility>
#include <set>
#include <string>
#include <map>
#include "version.h"
#include "polyhedron.h"

namespace gfe {

using PointArray = std::array<poly::Point, poly::nDims+1>;

// Poor man's graph structure for diagnostics only
template <typename T>
class GraphNode {
public:
    T value;
    std::vector<GraphNode<T> > children;
    GraphNode(T v) : value(v) {};
};

// Cell could probably benefit from having a virtual base class
class Cell {
public:
    using IdArray = std::array<int, poly::nDims+1>;
    using IdSet = std::set<int>;
    IdArray vertices;
    IdArray neighbors;
    IdSet influencers; // List of influencing vertices
};

class Mesh {
public:
    size_t dim;
    std::vector<poly::Point> vertices;
    std::vector<Cell> cells;
    std::vector<double> pittewayVolume() const;
    std::vector<double> volume() const;
    poly::Point cellCircumcenter(int index) const;
    void computeInfluencers();
    GraphNode<size_t> computeInfluencersStatistics();
    PointArray cell(int index) const;
    PointArray facetNormals(int index) const;
    void propagate(const poly::Point& center, const Cell& first, int cellIndex);
    void propagateRestricted(const poly::Point& center, const Cell& first, int cellIndex);
    void propagateStatistics(const poly::Point& center, const Cell& first, int cellIndex, GraphNode<size_t>& parent);
    void findNeighbors(const std::vector<std::vector<int>>& belongsTo);
    std::vector<double> volume3voro() const;
private:
    std::vector<double> pittewayVolume2() const;
    std::vector<double> pittewayVolume3() const;
    std::vector<double> volume2() const;
    std::vector<double> volume3() const;
};

std::istream& readGmsh(std::istream& in, Mesh &mesh);
std::istream& readFE(std::istream& in, Mesh &mesh);
std::ostream& writeVector(std::ostream& out, const std::vector<double>& vec);
std::ostream& operator<<(std::ostream& out, const Cell& cell);

} // namspace gfe

#endif // GIRAFFE_H
