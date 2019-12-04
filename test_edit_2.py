import open3d as o3d

# TODO doesn't work
# class NewVis(o3d.visualization.VisualizerWithEditing,
#              o3d.visualization.VisualizerWithKeyCallback):
#     def __init__(self):
#         super(NewVis).__init__()


class Test:
    def __init__(self, pcd):
        self.points = set()
        self.pcd = pcd
        self.pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    def __call__(self, vis):
        psi = vis.get_picked_points()
        for pi in psi:
            if pi not in self.points:
                print('New point {}'.format(pi))
                print(self.pcd.points[pi])
                print(self.pcd.colors[pi])
                self.points.add(pi)
                cs = self.pcd.points[pi]
                r = 0.1
                n, nsi, ds = self.pcd_tree.search_radius_vector_3d(cs, r)
                print('n_neighbours: {}'.format(n))
                for ni in nsi:
                    self.pcd.colors[ni] = [1, 0, 0]
                self.pcd.colors[pi] = [0, 1, 0]
        return False


if __name__ == "__main__":
    file_path = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    t = Test(pcd)
    while True:
        vis = o3d.visualization.VisualizerWithEditing()
        vis.register_animation_callback(t)
        vis.create_window(width=500, height=500, left=800, top=50)
        vis.add_geometry(t.pcd)
        vis.run()
        vis.destroy_window()
