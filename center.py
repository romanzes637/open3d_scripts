import open3d as o3d
import numpy as np

if __name__ == "__main__":
    file_path = 'conferenceRoom_1.xyzrgb'
    pcd = o3d.io.read_point_cloud(file_path)
    vis = o3d.visualization.VisualizerWithEditing()
    # color bug fixing xyzrgb (float rgb?)
    if np.max(pcd.colors) > 1:
        pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)/255)
    pcd.points = o3d.utility.Vector3dVector(
        pcd.points - np.mean(pcd.points, axis=0, keepdims=True))
    o3d.io.write_point_cloud("center.ply", pcd, write_ascii=True)
