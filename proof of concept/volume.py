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
import mpl_toolkits.mplot3d as a3
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import timeit

def plot_polyhedron_faces(faces):
    ax = a3.Axes3D(plt.figure())
    tri = a3.art3d.Poly3DCollection(faces)
    tri.set_alpha(0.3)
    ax.add_collection3d(tri)
    plt.show()

def volume(faces,vert):
    a = 0
    va = vert[a]
    volume = 0
    for face in faces:
        if a not in face:
            b = face[0]
            vb = vert[b]
            for i in range(1,len(face)-1):
                c = face[i]
                d = face[i+1]
                vc = vert[c]
                vd = vert[d]
                cross = np.cross(vb-vd,vc-vd)
                dot = np.dot(va-vd,cross)
                volume += (1.0/6)*abs(dot)
    return volume

def hull_volume(vertices):
    hull = ConvexHull(vertices)
    return hull.volume

vertices = [[ 0.  ,  0.  ,  0.  ],
            [ 0.5 ,  0.  ,  0.  ],
            [ 0.5 ,  0.5 ,  0.  ],
            [ 0.5 ,  0.  ,  0.5 ],
            [ 0.  ,  0.5 ,  0.  ],
            [ 0.  ,  0.5 ,  0.5 ],
            [ 0.  ,  0.  ,  0.5 ]]

verticesarr = [np.array(v) for v in vertices]

faces = [[0,1,2,4],
         [0,1,3,6],
         [0,6,5,4],
         [6,3,5],
         [3,2,5],
         [1,2,3],
         [4,5,2]]

faces_coords = []

for f in faces:
    coords = []
    for v in f:
        coords.append(vertices[v])
    faces_coords.append(coords)


if __name__ == '__main__':
    print(timeit.timeit("hull_volume(vertices)", setup="from __main__ import hull_volume, vertices", number=10000))
    print(timeit.timeit("volume(faces,verticesarr)", setup="from __main__ import volume, verticesarr, faces", number=10000))

    print(volume(faces,verticesarr))
    plot_polyhedron_faces(faces_coords[3:])
