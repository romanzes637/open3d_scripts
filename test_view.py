import open3d as o3d

if __name__ == "__main__":
    file_path = 'center.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    o3d.visualization.draw_geometries([pcd])
