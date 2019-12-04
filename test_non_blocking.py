import open3d as o3d

if __name__ == "__main__":
    file_path = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    for i in range(len(pcd.points)):
        print(i)
        pcd.colors[i] = [1, 1, 1]
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
    # vis.destroy_window()
