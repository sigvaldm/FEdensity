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
import copy

epsilon = 1e-10

def plane_line_intersection(line, p, n):
    v1 = line[0]
    v2 = line[1]
    normal_component = np.dot(v2-v1,n)

    is_parallel = abs(normal_component) < epsilon
    if is_parallel:
        return None

    alpha = np.dot(p-v1,n) / normal_component

    intersects = alpha>0 and alpha<1
    if not intersects:
        return None

    intersecting_point = v1+alpha*(v2-v1)
    return intersecting_point

def is_behind_plane(point, p, n):
    normal_component = np.dot(point-p,n)
    return normal_component < 0

class Polyhedron(object):
    def __init__(self, *vertices):
        self.vertices = []
        self.edges = [] # between which vertices?
        self.faces = [] # containing which edges?
        self.edge_faces = [] # which faces shares the edges?

        assert len(vertices)==4

        for vertex in vertices:
            self.vertices.append(np.array(vertex,float))


        self.edges = list(combinations(range(len(self.vertices)),2))
        self.edges = [list(a) for a in self.edges]

        # Face i is opposite vertex i
        self.faces.append([3,4,5])
        self.faces.append([1,2,5])
        self.faces.append([0,2,4])
        self.faces.append([0,1,3])

        # Which faces shares the given edge
        self.edge_faces.append([2,3])
        self.edge_faces.append([1,3])
        self.edge_faces.append([1,2])
        self.edge_faces.append([0,3])
        self.edge_faces.append([0,2])
        self.edge_faces.append([0,1])

    def __repr__(self):
        s = "Polyhedron\n\n"
        for i,f in enumerate(self.faces):
            s += "face %d:\n"%i
            for e in f:
                s += "  edge %d"%e
                vertex_ids = self.edges[e]
                vertices = [self.vertices[v] for v in vertex_ids]
                s += " = vertices %d-%d"%(vertex_ids[0],vertex_ids[1])
                s += " = "
                s += vertices[0].__repr__()
                s += " - "
                s += vertices[1].__repr__()
                s += "\n"
            s += "\n"
        for i,e in enumerate(self.edge_faces):
            s += "edge %d is shared between faces:"%i
            for elem in e:
                s += " %d"%(elem)
            s += "\n"
        return s

    def plot(self):
        ax = a3.Axes3D(plt.figure())
        tri = a3.art3d.Poly3DCollection(list(combinations(self.vertices, 3)))
        tri.set_alpha(0.3)
        ax.add_collection3d(tri)
        plt.show()

    def clip_naive(self, p, n):
        edges = list(combinations(self.vertices, 2))
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
                    self.vertices.append(vn)

        for i,v in enumerate(self.vertices):
            if np.dot(v-p,n)>epsilon:
                self.vertices.pop(i)

    def find_intersecting_edge(self, p, n):
        for i,edge in enumerate(self.edges):
            edge = [self.vertices[e] for e in edge]
            vn = plane_line_intersection(edge, p, n)
            if vn is not None:
                return i

    def clip(self, p, n):
        first_anchor_edge_id = self.find_intersecting_edge(p, n)
        anchor_edge_id = first_anchor_edge_id
        while True:
            face_id = self.edge_faces[anchor_edge_id][0]
            updated_face = []
            new_edge = []

            for edge_id in self.faces[face_id]:
                edge = self.edges[edge_id]
                edge_vertices = [self.vertices[i] for i in edge]
                new_vertex = plane_line_intersection(edge_vertices, p, n)
                if new_vertex is None:
                    if is_behind_plane(edge_vertices[0], p, n):
                        updated_face.append(edge_id)
                    else:
                        edge_faces.remove(face_id)
                else:
                    self.vertices.append(new_vertex)
                    new_vertex_id = len(self.vertices)-1
                    if is_behind_plane(edge_vertices[0], p, n):
                        self.edges.append([edge[0],new_vertex_id])
                    else:
                        self.edges.append([edge[1],new_vertex_id])
                    self.edge_faces.append([face_id])
                    self.edge_faces[edge_id].remove(face_id)
                    updated_face.append(len(self.edges)-1)
                    new_edge.append(new_vertex_id)
                    if edge_id != anchor_edge_id:
                        next_anchor_edge_id = edge_id
            if new_edge != []:
                self.edges.append(new_edge)
                self.edge_faces.append([face_id])
                updated_face.append(len(self.edges)-1)
                self.faces[face_id] = updated_face

            anchor_edge_id = next_anchor_edge_id
            if anchor_edge_id == first_anchor_edge_id:
                break

    def intersect_edge(self, edge_id, p, n):
        """
        Given the index of an edge "edge_id" and a pair (p,n) describing the
        plane, intersects this edge by the plane and creates a new edge if
        necessary. Returns the id of the intersected edge in the self.edges
        list, or None if the whole edge is to be cut away. Note: The old edge
        stile exists in the list.
        """
        edge = self.edges[edge_id]
        edge_vertices = [self.vertices[i] for i in edge]
        new_vertex = plane_line_intersection(edge_vertices, p, n)
        if new_vertex is None:
            if is_behind_plane(edge_vertices[0], p, n):
                return edge_id, None
            else:
                return None, None
        else:
            self.vertices.append(new_vertex)
            new_vertex_id = len(self.vertices)-1
            if is_behind_plane(edge_vertices[0], p, n):
                self.edges.append([edge[0],new_vertex_id])
            else:
                self.edges.append([edge[1],new_vertex_id])
            self.edge_faces.append([])
            new_edge_id = len(self.edges)-1
            return new_edge_id, new_vertex_id

    def clip2(self, p, n):
        first_anchor_edge_id = self.find_intersecting_edge(p, n)
        anchor_edge_id = first_anchor_edge_id

        while True:
            face_id = self.edge_faces[anchor_edge_id][0]
            updated_face = []
            new_edge = []

            for edge_id in self.faces[face_id]:
                edge = self.edges[edge_id]
                edge_vertices = [self.vertices[i] for i in edge]

                new_edge_id, new_vertex_id = self.intersect_edge(edge_id, p, n)

                if new_vertex_id:
                    new_edge.append(new_vertex_id)

                self.edge_faces[edge_id].remove(face_id)

                if new_edge_id:
                    updated_face.append(new_edge_id)
                    self.edge_faces[new_edge_id].append(face_id)

                if edge_id != anchor_edge_id:
                    next_anchor_edge_id = edge_id

            if new_edge != []:
                self.edges.append(new_edge)
                self.edge_faces.append([face_id])
                updated_face.append(len(self.edges)-1)

            self.faces[face_id] = updated_face

            # print("vertices:")
            # print(np.array(self.vertices))

            print(self)

            anchor_edge_id = next_anchor_edge_id
            if anchor_edge_id == first_anchor_edge_id:
                break

    def replace_face_edge(self, face_id, edge_id, new_edge_id):

        self.faces[face_id].remove(edge_id)
        self.edge_faces[edge_id].remove(face_id)

        if new_edge_id is not None:
            self.faces[face_id].append(new_edge_id)
            self.edge_faces[new_edge_id].append(face_id)

    def clip3(self, p, n):
        first_anchor_edge_id = self.find_intersecting_edge(p, n)
        anchor_edge_id = first_anchor_edge_id

        replacement_anchor_edge_id, asdf = self.intersect_edge(anchor_edge_id, p, n)

        while True:
            face_id = self.edge_faces[anchor_edge_id][0]
            face = self.faces[face_id]

            face_ = copy.deepcopy(face)
            self.replace_face_edge(face_id, anchor_edge_id, replacement_anchor_edge_id)

            for edge_id in face_:
                if edge_id != anchor_edge_id:
                    new_edge_id, new_vertex_id = self.intersect_edge(edge_id, p, n)
                    self.replace_face_edge(face_id, edge_id, new_edge_id)
                    if new_vertex_id:
                        next_anchor_edge_id = edge_id
                        replacement_anchor_edge_id = new_edge_id

    # Don't use. I think it has bad consequences
    def replace_edge(self, edge_id, new_edge_id):
        if edge_id != new_edge_id:
            for face_id in self.edge_faces[edge_id]:
                self.faces[face_id].remove(edge_id)

            if new_edge_id is not None:
                for face_id in self.edge_faces[edge_id]:
                    self.faces[face_id].append(new_edge_id)
                    self.edge_faces[new_edge_id].append(face_id)

            self.edge_faces[edge_id] = []

    def clip4(self, p, n):
        # 1. Anchor edge = some intersecting edge
        # 2. Face = a face of anchor edge
        # 3. Cut anchor edge & update face
        # 4. For all edge = non-anchor-edges in face:
        #       a. New edge = this
        #       b. If new vertex: next anchor edge = this edge

vertices = [np.array([0,0,0]),
            np.array([1,0,0]),
            np.array([0,1,0]),
            np.array([0,0,1])]

p = Polyhedron(*vertices)
# p.clip3([0.5,0,1],[1,0,0])
print(p)
