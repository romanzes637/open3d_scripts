# examples/Python/Basic/kdtree.py

import numpy as np
import open3d as o3d

if __name__ == "__main__":

    file_name = 'plane.pts'
    print("Testing kdtree in open3d ...")
    print("Load a point cloud and paint it gray.")
    pcd = o3d.io.read_point_cloud(file_name)
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    pi = 4664
    r = 20
    nn = 300
    print("Paint the {}th point red.".format(pi))
    pcd.colors[pi] = [1, 0, 0]
    print("Find its {} nearest neighbors, paint blue.".format(nn))
    n, idxs, ds = pcd_tree.search_knn_vector_3d(pcd.points[pi], nn)
    np.asarray(pcd.colors)[idxs[1:], :] = [0, 0, 1]
    print("Find its neighbors with distance less than {}, paint green.".format(r))
    n, idxs, ds = pcd_tree.search_radius_vector_3d(pcd.points[pi], r)
    print('n_neighbours: {}'.format(n))
    np.asarray(pcd.colors)[idxs[1:], :] = [0, 1, 0]
    print("Visualize the point cloud.")
    o3d.visualization.draw_geometries_with_editing([pcd])
    print("")