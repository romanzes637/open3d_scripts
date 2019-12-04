import open3d as o3d
import time

vis = o3d.visualization.Visualizer()
vis.create_window()
file_path = 'cropped_1.ply'
pcd = o3d.io.read_point_cloud(file_path)

vis.add_geometry(pcd)
vis.update_geometry()
vis.update_renderer()
vis.poll_events()
vis.run()

# The point cloud is now shown

pcd.points = o3d.utility.Vector3dVector([])
pcd.colors = o3d.utility.Vector3dVector([])

vis.update_geometry()
vis.update_renderer()
vis.poll_events()
vis.run()

# The point cloud is no longer shown

time.sleep(5)