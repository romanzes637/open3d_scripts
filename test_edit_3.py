import open3d as o3d


if __name__ == "__main__":
    file_path = 'cropped_2.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    r = 0.1
    while True:
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(width=500, height=500, left=800, top=50)
        vis.add_geometry(pcd)
        vis.run()
        vis.destroy_window()
        # Paint point green and neighbours red
        psi = vis.get_picked_points()
        for pi in psi:
            cs = pcd.points[pi]
            n, nsi, ds = pcd_tree.search_radius_vector_3d(cs, r)
            print('n_neighbours: {}'.format(n))
            for ni in nsi:
                pcd.colors[ni] = [1, 0, 0]
            pcd.colors[pi] = [0, 1, 0]
