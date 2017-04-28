/**
 * @file		FEdensity.h
 * @brief		FEdensity API
 * @author		Sigvald Marholm <sigvaldm@fys.uio.no>,
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

#include <vector>
#include <array>
#include <string>
#include "version.h"
#include "polyhedron.h"

namespace fedensity {

using std::vector;
using std::array;
using std::string;
using poly::Point;
using poly::nDims;

class Cell {
public:
    array<int, nDims+1> vertices;
    // array<Cell*, nDims+1> neighbors;
};

class Mesh {
public:
    vector<double> pittewayVolume() const;
    vector<Point> vertices;
    vector<Cell> cells;
};

Mesh readGmsh(const string& filename);
void writeVector(const string& filename, const vector<double>& vec);

} // namspace fedensity
