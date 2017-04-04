import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d as a3
from scipy.spatial import ConvexHull
from itertools import combinations
import timeit

epsilon = 1e-10

def plot_polyhedron(vertices):
    ax = a3.Axes3D(plt.figure())
    tri = a3.art3d.Poly3DCollection(list(combinations(vertices, 3)))
    tri.set_alpha(0.3)
    ax.add_collection3d(tri)
    plt.show()

# Naïve. Tests all pairs of vertices as edges, which produces 14 vertices,
# 7 of which is not actually vertices but points inside the same convex hull.
def clip_polyhedron(vertices, p, n):
    edges = list(combinations(vertices, 2))
    for edge in edges:
        v1 = edge[0]
        v2 = edge[1]
        denominator = np.dot(v2-v1,n)
        if abs(denominator) > epsilon:
            # If the edge is parallell to clipping wall it either never
            # intersects or it's already inside the wall in which case we don't
            # need to append a new vertex intersecting with the wall.
            alpha = np.dot(p-v1,n) / denominator

            # If edge goes through the plane, add vertex
            if alpha>0 and alpha<1:
                vn = v1+alpha*(v2-v1)
                vertices.append(vn)

    for i,v in enumerate(vertices):
        if np.dot(v-p,n)>epsilon:
            vertices.pop(i)

# Keeps track of edges and only cuts previous edges. Does not keep track of
# faces. Vertices produced are 8 (7 of which are actually vertices).
# Computational time halved compared with clip_polyhedron(). Since it does not
# keep track of faces convex hull algorithm must be used to compute volume.
def clip_polyhedron2(vertices, edges, p, n):
    delete_vertices = set()
    old_vertices = len(vertices)
    for i, edge in enumerate(edges):
        v1 = vertices[edge[0]]
        v2 = vertices[edge[1]]
        denominator = np.dot(v2-v1,n)

        if abs(denominator) > epsilon:
            # If the edge is parallel to clipping wall it either never
            # intersects or it's already inside the wall in which case we don't
            # need to append a new vertex intersecting with the wall.

            numerator = np.dot(p-v1,n)
            alpha = numerator / denominator

            # If edge goes through the plane, add vertex
            if alpha>0 and alpha<1:
                vn = v1+alpha*(v2-v1)
                vertices.append(vn)

                # Update shortened edges
                if numerator>0:
                    delete_vertices.add(edge[1]) # These v's are actually pointers to elements in a list
                    edge[1] = len(vertices)-1
                else:
                    delete_vertices.add(edge[0])
                    edge[0] = len(vertices)-1

    new_edges = list(combinations(range(old_vertices,len(vertices)),2))
    new_edges = [list(e) for e in new_edges]
    edges.extend(new_edges)

    for i in delete_vertices:
        vertices.pop(i)
        for edge in edges:
            for k in range(2):
                if edge[k]>i: edge[k] = edge[k] - 1

def method1():
    vertices = [np.array([0,0,0]),
                np.array([1,0,0]),
                np.array([0,1,0]),
                np.array([0,0,1])]

    clip_polyhedron(vertices,[0.5,0,0],[1,0,0])
    clip_polyhedron(vertices,[0,0.5,0],[0,1,0])
    clip_polyhedron(vertices,[0,0,0.5],[0,0,1])

    hull = ConvexHull(vertices)
    return vertices, hull.volume

def method2():

    vertices = [np.array([0,0,0]),
                np.array([1,0,0]),
                np.array([0,1,0]),
                np.array([0,0,1])]

    edges = list(combinations(range(len(vertices)),2))
    edges = [list(e) for e in edges]

    clip_polyhedron2(vertices,edges,[0.5,0,0],[1,0,0])
    clip_polyhedron2(vertices,edges,[0,0.5,0],[0,1,0])
    clip_polyhedron2(vertices,edges,[0,0,0.5],[0,0,1])

    hull = ConvexHull(vertices)
    return vertices, hull.volume


if __name__ == '__main__':
    print(timeit.timeit("method1()", setup="from __main__ import method1", number=1000))
    print(timeit.timeit("method2()", setup="from __main__ import method2", number=1000))

    vertices, volume = method2()
    print np.array(vertices)
    plot_polyhedron(vertices)
