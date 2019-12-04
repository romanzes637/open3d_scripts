import open3d as o3d


def add_coord_axes(vis):
    opt = vis.get_render_option()
    opt.show_coordinate_frame = not opt.show_coordinate_frame
    return False


if __name__ == '__main__':
    file_path = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_path)
    key_to_callback = dict()
    key_to_callback[ord("Q")] = add_coord_axes
    o3d.visualization.draw_geometries_with_key_callbacks([pcd], key_to_callback)