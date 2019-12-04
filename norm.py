import open3d as o3d
import numpy as np

if __name__ == "__main__":
    file_path = 'conferenceRoom_1.xyzrgb'
    # norm_type = 'coord_max'
    norm_type = 'abs_max'
    pcd = o3d.io.read_point_cloud(file_path)
    vis = o3d.visualization.VisualizerWithEditing()
    # color bug fixing xyzrgb (float rgb?)
    if np.max(pcd.colors) > 1:
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)/255)
    points_min = np.min(pcd.points, axis=0)
    points_ptp = np.ptp(pcd.points, axis=0)  # max - min
    print(points_min, points_ptp)
    if norm_type == 'coord_max':
        pcd.points = o3d.utility.Vector3dVector(
            (pcd.points - points_min) / points_ptp)
    else:
        pcd.points = o3d.utility.Vector3dVector(
            (pcd.points - points_min) / max(points_ptp))
    print(np.min(pcd.points), np.max(pcd.points))
    o3d.io.write_point_cloud("norm.ply", pcd, write_ascii=True)
