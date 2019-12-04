import open3d as o3d
import multiprocessing as mp
import time
import numpy as np
# import transforms3d.euler as txe
# from multiprocessing.Q import  as queue_empty


class Viewer(object):
    def __init__(self):
        self.q = mp.Queue()

    def worker(self, q):
        for _ in range(5):
            time.sleep(10)
            T = np.eye(4)
            # T[:3, :3] = txe.euler2mat(np.deg2rad(20), 0, 0)
            q.put(T)
            print('put')
        q.put(None)  # poison pill

    def run(self):
        pcd = o3d.io.read_point_cloud('cropped_1.ply')
        vis = o3d.visualization.Visualizer()
        vis.create_window('cloud', width=640, height=480)
        vis.add_geometry(pcd)

        p = mp.Process(target=self.worker, args=(self.q,))
        p.start()

        keep_running = True
        while keep_running:
            try:
                T = self.q.get(block=False)
                print(T)
                if T is not None:
                    print('got T')
                    print(T)
                    # pcd.transform(T)
                # else:
                #     print('got poison. dying')
                #     keep_running = False
                vis.update_geometry()
            except Exception as e:
                print(e)
                # pass
            vis.update_renderer()
            keep_running = keep_running and vis.poll_events()
        vis.destroy_window()

        p.join()


if __name__ == '__main__':
    v = Viewer()
    v.run()
