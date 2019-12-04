import open3d as o3d
import numpy as np


if __name__ == "__main__":
    # file_path = 'conferenceRoom_1.xyzrgb'
    file_path = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    # o3d.visualization.draw_geometries_with_editing([pcd])
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.get_render_option()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    indices = vis.get_picked_points()
    points = np.take(pcd.points, indices, axis=0)
    print(list(zip(indices, points)))
    new_pcd = o3d.geometry.PointCloud()
    new_pcd.points = o3d.utility.Vector3dVector(points)
    if pcd.has_colors:
        if len(pcd.colors) > 0:
            colors = np.take(pcd.colors, indices, axis=0)
            new_pcd.colors = o3d.utility.Vector3dVector(colors)
    o3d.io.write_point_cloud("edit.ply", new_pcd, write_ascii=True)

