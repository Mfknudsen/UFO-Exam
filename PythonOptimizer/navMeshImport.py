from typing import List
from dataclasses import dataclass
from vectors import vector3

@dataclass
class NavMeshImport:

    clean_point: vector3
    indices: List[int]
    vertices: List[vector3]
    final_vertex_count: int
    final_triangle_count: int
    final_indices_count: int

    def get_vertices(self) -> List[vector3]:
        return self.vertices

    def get_indices(self) -> List[int]:
        return self.indices

    def get_clean_point(self) -> vector3:
        return self.clean_point

    def f_vertex(self) -> int:
        return self.final_vertex_count

    def f_triangle(self) -> int:
        return self.final_triangle_count

    def f_indices(self) -> int:
        return self.final_indices_count
