import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib import cm

# points = np.loadtxt('plane.pts').astype(np.float32)
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(points)
# o3d.io.write_point_cloud("plane.ply", pcd, write_ascii=True)
# o3d.visualization.draw_geometries([pcd])

if __name__ == "__main__":
    file_path = 'plane.pts'

    print("Load a point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud(file_path)
    print(pcd)
    print(dir(pcd))
    # nd = pcd.compute_nearest_neighbor_distance()
    # print(nd)
    # print(pcd.compute_mean_and_covariance())
    # points_clusters = pcd.cluster_dbscan(eps=25,
    #                                      min_points=2, print_progress=True)
    # plt.hist(points_clusters, bins='auto')
    # plt.show()
    # # print(np.asarray(pcd.points))
    # clusters_points = {}
    # for i, c in enumerate(points_clusters):
    #     clusters_points.setdefault(c, []).append(i)
    # noise = clusters_points.pop(-1, None)
    # pcds = list()
    # # Noise
    # colors = np.zeros((len(pcd.points), 3))
    # if noise is not None:
    #     points = [pcd.points[x] for x in noise]
    #     noise_pcd = o3d.geometry.PointCloud()
    #     noise_pcd.points = o3d.utility.Vector3dVector(points)
    #     noise_pcd.paint_uniform_color([0, 0, 0])  # black
    #     pcds.append(noise_pcd)
    # cmap = cm.get_cmap('jet')
    # n_clusters = len(clusters_points)
    # print('n_clusters: {}'.format(len(clusters_points)))
    # for k, v in clusters_points.items():
    #     points = [pcd.points[x] for x in v]
    #     cluster_pcd = o3d.geometry.PointCloud()
    #     cluster_pcd.points = o3d.utility.Vector3dVector(points)
    #     c = cmap(k/n_clusters)
    #     cluster_pcd.paint_uniform_color(c[:-1])  # remove alpha
    #     for x in v:
    #         colors[x] = c[:-1]
    #     pcds.append(cluster_pcd)
    # # All
    # pcd.colors = o3d.utility.Vector3dVector(colors)
    # o3d.io.write_point_cloud("plain_clusters.ply", pcd, write_ascii=True)
    # o3d.visualization.draw_geometries(
    #     [mesh_box + mesh_sphere + mesh_cylinder + mesh_frame])

    # print("Let\'s draw a cubic using o3d.geometry.LineSet")
    # points = [[0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0], [0, 0, 1], [1, 0, 1],
    #           [0, 1, 1], [1, 1, 1]]
    # lines = [[0, 1], [0, 2], [1, 3], [2, 3], [4, 5], [4, 6], [5, 7], [6, 7],
    #          [0, 4], [1, 5], [2, 6], [3, 7]]
    # colors = [[1, 0, 0] for i in range(len(lines))]
    # line_set = o3d.geometry.LineSet()
    # line_set.points = o3d.utility.Vector3dVector(points)
    # line_set.lines = o3d.utility.Vector2iVector(lines)
    # line_set.colors = o3d.utility.Vector3dVector(colors)
    # o3d.visualization.draw_geometries([line_set])

    # print("Recompute the normal of the point cloud")
    # pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    #     radius=10, max_nn=30))
    # # o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries_with_editing([pcd])
    # print("Downsample the point cloud with a voxel of 0.05")
    # downpcd = pcd.voxel_down_sample(voxel_size=1)
    # o3d.visualization.draw_geometries([downpcd])

    # print("Recompute the normal of the downsampled point cloud")
    # downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    #     radius=10, max_nn=30))
    # o3d.visualization.draw_geometries([downpcd])

    # print("Print a normal vector of the 0th point")
    # print(downpcd.normals[0])
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(downpcd.normals)[:10, :])
    # print("")

    # print("Load a polygon volume and use it to crop the original point cloud")
    # vol = o3d.visualization.read_selection_polygon_volume(
    #     "../../TestData/Crop/cropped.json")
    # chair = vol.crop_point_cloud(pcd)
    # o3d.visualization.draw_geometries([chair])
    # print("")
    #
    # print("Paint chair")
    # chair.paint_uniform_color([1, 0.706, 0])
    # o3d.visualization.draw_geometries([chair])
    # print("")
