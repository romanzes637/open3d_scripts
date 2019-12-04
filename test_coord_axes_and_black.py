import open3d as o3d
import numpy as np


def add_coord_axes(vis):
    opt = vis.get_render_option()
    opt.show_coordinate_frame = not opt.show_coordinate_frame
    return False


def change_background_to_black(vis):
    opt = vis.get_render_option()
    prev_color = opt.background_color
    if np.array_equal(prev_color, [1, 1, 1]):
        opt.background_color = np.asarray([0, 0, 0])
    else:
        opt.background_color = np.asarray([1, 1, 1])
    return False


if __name__ == '__main__':
    file_path = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    key_to_callback = dict()
    key_to_callback[ord("5")] = change_background_to_black
    key_to_callback[ord("6")] = add_coord_axes
    o3d.visualization.draw_geometries_with_key_callbacks([pcd], key_to_callback)