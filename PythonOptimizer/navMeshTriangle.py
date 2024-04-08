from typing import List
from dataclasses import dataclass
from vectors import vector3

@dataclass
class NavMeshTriangle:
    id: int
    a: int
    b: int
    c: int
    neighbor_ids: List[int] = None
    width_distance_between_neighbor: List[float] = None

    def __post_init__(self):
        if self.neighbor_ids is None:
            self.neighbor_ids = []
        if self.width_distance_between_neighbor is None:
            self.width_distance_between_neighbor = []

    def vertices(self) -> List[int]:
        return [self.a, self.b, self.c]

    def neighbors(self) -> List[int]:
        return self.neighbor_ids

    def set_neighbor_ids(self, neighbor_ids: List[int]):
        self.neighbor_ids.clear()
        for t in neighbor_ids:
            if t not in self.neighbor_ids:
                self.neighbor_ids.append(t)

    def set_border_width(self, verts: List[vector3], triangles: List['NavMeshTriangle']):
        if not self.neighbor_ids:
            return

        for i in range(len(self.neighbor_ids)):
            self.width_distance_between_neighbor.append(0.0)

        for i in range(len(self.neighbor_ids)):
            other_id = self.neighbor_ids[i]
            other = triangles[other_id]
            ids = self.shared_vertices(other)

            if len(ids) != 2:
                continue

            dist = vector3.distance(verts[ids[0]], verts[ids[1]])

            if i + 2 < len(self.neighbor_ids):
                connected_border_neighbor = -1
                if other_id in triangles[self.neighbor_ids[i + 2]].neighbor_ids:
                    connected_border_neighbor = i + 2
                elif other_id in triangles[self.neighbor_ids[i + 1]].neighbor_ids:
                    connected_border_neighbor = i + 1

                if connected_border_neighbor > -1:
                    ids = triangles[self.neighbor_ids[connected_border_neighbor]].shared_vertices(self)
                    if len(ids) == 2:
                        dist += vector3.distance(verts[ids[0]], verts[ids[1]])
                        self.width_distance_between_neighbor[connected_border_neighbor] = dist
            elif i + 1 < len(self.neighbor_ids):
                if other_id in triangles[self.neighbor_ids[i + 1]].neighbor_ids:
                    ids = triangles[self.neighbor_ids[i + 1]].shared_vertices(self)
                    if len(ids) == 2:
                        dist += vector3.distance(verts[ids[0]], verts[ids[1]])
                        self.width_distance_between_neighbor[i + 1] = dist

            self.width_distance_between_neighbor[i] = dist

    def shared_vertices(self, other: 'NavMeshTriangle') -> List[int]:
        shared = []
        for v in self.vertices():
            if v in other.vertices():
                shared.append(v)
        return shared
