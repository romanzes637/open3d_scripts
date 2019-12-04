import open3d as o3d

if __name__ == "__main__":
    file_path = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    o3d.io.write_point_cloud(file_path, pcd, write_ascii=True)
