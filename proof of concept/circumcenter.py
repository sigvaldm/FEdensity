"""
Copyright 2017 Sigvald Marholm <marholm@marebakken.com>

This file is part of FEdensity.

FEdensity is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

FEdensity is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
details.

You should have received a copy of the GNU General Public License along with
FEdensity. If not, see <http://www.gnu.org/licenses/>.
"""

from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3
from scipy.spatial import ConvexHull
from itertools import combinations
import timeit
from dolfin import *

def circumcircle(v):
    n1 = v[1]-v[0]
    n2 = v[2]-v[0]
    p1 = 0.5*(v[1]+v[0])
    p2 = 0.5*(v[2]+v[0])
    A = np.array([n1,n2])
    b = np.array([np.dot(n1,p1),np.dot(n2,p2)])
    center = np.linalg.solve(A,b)
    radius = np.linalg.norm(v[0]-center)
    return center, radius

def isPitteway(mesh):
    coords = mesh.coordinates()
    isPitteway_ = True
    for cell in cells(mesh):
        v = cell.entities(0)
        v = [coords[i] for i in v]
        center, radius = circumcircle(v)
        if not cell.contains(Point(center)): isPitteway_ = False
    return isPitteway_

def isDelaunay(mesh):
    coords = mesh.coordinates()
    isDelaunay_ = True
    for k,cell in enumerate(cells(mesh)):
        # print("Cell %d/%d"%(k,mesh.num_cells()))
        vi = cell.entities(0)
        v  = [coords[i] for i in vi]
        center, radius = circumcircle(v)
        for i, coord in enumerate(coords):
            if i not in vi:
                if np.linalg.norm(coord-center)<radius: isDelaunay_ = False
    return isDelaunay_

def circumcenters(mesh):
    circumcenters = []
    coords = mesh.coordinates()
    for cell in cells(mesh):
        vertex_indices = cell.entities(0)
        v = [coords[i] for i in vertex_indices]
        center, radius = circumcircle(v)
        circumcenters.append(center)
    return circumcenters

files = ['circle.xml','circuit.xml','nonuniform.xml']
for f in files:
    mesh = Mesh("../mesh/"+f)
    # mesh = UnitSquareMesh(8,8)
    print("mesh:",f)
    print("isDelaunay:",isDelaunay(mesh))
    print("isPitteway:",isPitteway(mesh))
