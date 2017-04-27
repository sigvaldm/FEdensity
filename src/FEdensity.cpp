/**
 * @file		FEdensity.cpp
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

#include "FEdensity.h"
using poly::Point;
using poly::Polyhedron;

vector<double> Mesh::pittewayVolume() const{

    vector<double> volume(vertices.size());

    for(const auto& cell : cells){
        array<Point, nDims+1> vs;
        for(int i=0; i<nDims+1; i++) vs[i] = vertices[cell.vertices[i]];

        for(int i=0; i<nDims+1; i++){
            Polyhedron p(vs);

            for(int j=0; j<nDims+1; j++){
                if(j!=i){
                    Point point = 0.5*(vertices[j]+vertices[i]);
                    Point normal = vertices[j]-vertices[i];
                    p.clip(point,normal);
                }
            }

            volume[i] += p.volume();

        }
    }

    return volume;
}
