import numpy as np
import open3d as o3d

if __name__ == "__main__":
    points = [[0, 0, 0], [1, 0, 0], [1, 2, 0], [0, 2, 0],
              [0, 0, 3], [1, 0, 3], [1, 2, 3], [0, 2, 3]]
    triangles = [[0, 2, 1], [0, 3, 2],  # -Z
                 [2, 5, 1], [5, 2, 6],  # X
                 [0, 4, 7], [0, 7, 3],  # -X
                 [0, 1, 5], [0, 5, 4],  # -Y
                 [2, 3, 7], [2, 7, 6],  # Y
                 [4, 5, 6], [4, 6, 7]]  # Z
    colors = [[1, 0, 0] for i in range(len(points))]
    tri_mesh = o3d.geometry.TriangleMesh()
    tri_mesh.vertices = o3d.utility.Vector3dVector(points)
    tri_mesh.triangles = o3d.utility.Vector3iVector(triangles)
    tri_mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    tri_mesh.compute_vertex_normals()
    tri_mesh.compute_triangle_normals()
    o3d.visualization.draw_geometries([tri_mesh])