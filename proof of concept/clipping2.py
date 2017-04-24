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

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3
from scipy.spatial import ConvexHull
from itertools import combinations
import timeit

epsilon = 1e-10

class Polyhedron(object):
    def __init__(self, *vertices):
        self.vertices = []
        self.faces = []

        for vertex in vertices:
            self.vertices.append(np.array(vertex,float))

        # Assume polyhedron is initialized to a simplex
        self.faces = list(combinations(range(len(self.vertices)),3))
        self.faces = [list(a) for a in self.faces]

    def find_intersecting_edge(self, n, p):

        # Find an edge going through the plane
        for face_id, face in enumerate(self.faces):
            for i in range(len(face)):
                id1 = face[i]
                id2 = face[(i+1)%len(face)]
                v1 = vertices[id1]
                v2 = vertices[id2]

                denom = np.dot(p-v1,n)
                if abs(denom) > epsilon:
                    alpha = np.dot(p-v1,n) / denom

                    # Edge goes through the plane
                    if alpha>0 and alpha<1:
                        return face_id

    def clip_face(self, n, p, face, delete_vertices):

        delete_vertices = set()
        new_vertices = []

        for i in range(len(face)):
            iplus = (i+1)%len(face)
            id1 = face[i]
            id2 = face[iplus]
            v1 = vertices[id1]
            v2 = vertices[id2]

            numerator   = np.dot(p -v1,n)
            denominator = np.dot(v2-v1,n)
            keep_id1 = numerator > 0

            # Make sure edge is not parallel to wall
            if abs(denominator) > epsilon:

                alpha = numerator / denominator

                # If edge goes through the plane, add vertex
                if alpha>0 and alpha<1:
                    vn = v1+alpha*(v2-v1)
                    new_vertices.append(vn)


    def clip(self, n, p):

        p = np.array(p)
        n = np.array(n)

        face_id = self.find_intersecting_edge(n, p):
        self.clip_face(n, p, face):
