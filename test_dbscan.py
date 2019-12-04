import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib import cm


if __name__ == "__main__":
    file_path = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    points_clusters = pcd.cluster_dbscan(eps=0.001,
                                         min_points=2, print_progress=True)
    n_clusters = len(set(points_clusters))
    print('n_clusters: {}'.format(n_clusters))
    plt.title('n_clusters: {}'.format(n_clusters))
    plt.hist(points_clusters, bins='auto')
    plt.show()
    colors = np.zeros((len(pcd.points), 3))
    cmap = cm.get_cmap('jet')
    for i, c in enumerate(points_clusters):
        if c != -1:  # noise cluster (black color by default)
            color = cmap(c / n_clusters)[:-1]  # remove alpha
            colors[i] = color
    pcd.colors = o3d.utility.Vector3dVector(colors)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=10, max_nn=30))
    o3d.io.write_point_cloud("dbscan.ply", pcd, write_ascii=True)
    o3d.visualization.draw_geometries([pcd])
