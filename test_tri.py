import open3d as o3d

if __name__ == "__main__":
    file_name = 'plane.ply'
    pcd = o3d.io.read_point_cloud(file_name)
    # CONSTRUCTORS
    # Convex hull (Bad quality!)
    # tm = pcd.compute_convex_hull()
    # Ball Pivoting
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=10, max_nn=30))
    tm = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd=pcd,
        radii=o3d.utility.DoubleVector(range(1, 50, 1)))
    # Alpha shape FIXME doesn't work
    # tn = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
    #     pcd=pcd, depth=8, width=0, scale=1.1, linear_fit=False)
    # Alpha shape FIXME doesn't work
    # tm = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(
    #     pcd=pcd, alpha=0.4, tetra_mesh=None, pt_map=None)
    # FILTERS
    filter_scope = o3d.geometry.FilterScope.All
    # Filter simple
    # tm = tm.filter_smooth_simple(number_of_iterations=1,
    #                              filter_scope=filter_scope)
    # Filter sharpen
    # tm = tm.filter_sharpen(1, strength=0.1, filter_scope=filter_scope)
    # Filter Laplacian
    # tm = tm.filter_smooth_laplacian(10, 0.5,
    #                                 filter_scope=filter_scope)
    # Filter Taubin FIXME doesn't work
    # tm = tm.smooth_taubin(1, 0.5, mu=-0.53,
    #                       filter_scope=filter_scope)
    tm.compute_vertex_normals()
    o3d.visualization.draw_geometries([tm, pcd])
