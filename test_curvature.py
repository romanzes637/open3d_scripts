import numpy as np
import open3d as o3d
from matplotlib import cm
import matplotlib.pyplot as plt


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


def as_cartesian(rthetaphi):
    # takes list rthetaphi (single coord)
    r = rthetaphi[0]
    # theta = rthetaphi[1]*pi/180  # to radian
    # phi = rthetaphi[2]*pi/180
    theta = rthetaphi[1]
    phi = rthetaphi[2]
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return [x, y, z]


if __name__ == "__main__":
    file_name = 'plane.pts'
    file_name = 'cropped_1.ply'
    pcd = o3d.io.read_point_cloud(file_name)
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(width=500, height=500, left=800, top=50)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()
    psi = vis.get_picked_points()
    # Main process
    # np.random.seed(33)
    # pi = np.random.randint(0, len(pcd.points))
    # pi = 5292
    # pi = 5294
    # pi = 5000
    # pi = 2000
    # pi = psi[0]
    # pcd.points = o3d.utility.Vector3dVector(np.unique(pcd.points, axis=0))
    # pcd.paint_uniform_color([0.5, 0.5, 0.5])
    # pcd.normals = o3d.utility.Vector3dVector(np.zeros_like(pcd.points))
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    n_steps = 6
    r = 0.1
    max_r = 0.5
    max_nn = 30
    max_steps = 10
    min_local_alpha = np.cos(np.pi/2)
    min_global_alpha = np.cos(np.pi/n_steps)
    min_beta = np.cos(np.pi)
    max_beta = np.cos(0)
    print(min_local_alpha, min_global_alpha, min_beta, max_beta)
    theta = np.linspace(0, np.pi, num=n_steps + 1)
    phi = np.linspace(0, 2 * np.pi, num=n_steps * 2 + 1)
    dps = []
    for t in theta:
        for p in phi:
            dp = as_cartesian([1.0, t, p])
            dps.append(dp)
    print(len(dps))
    # TODO find the best neighbour for each ray
    lines = []
    colors = []
    global_rs = []
    global_kappas = []
    for pi in psi:
        for j, dp in enumerate(dps):
            print('Ray: {}/{}'.format(j + 1, len(dps)))
            local_lines = []
            local_colors = []
            p0 = pi
            p1 = pi
            global_vec = dp
            global_norm = np.linalg.norm(global_vec)
            points = {pi}
            alphas = []
            betas = []
            gammas = []
            tangents = []
            binormals = []
            normals = []
            dists = []
            rs = []
            cmap = cm.get_cmap('RdYlGn')
            prev_n_lines = -1
            start_type = 1
            cnt = 0
            while cnt < max_steps:
                cnt += 1
                if len(local_lines) == prev_n_lines:
                    break
                prev_n_lines = len(local_lines)
                # n, idxs, ds = pcd_tree.search_knn_vector_3d(pcd.points[p0], max_nn)
                n, idxs, ds = pcd_tree.search_radius_vector_3d(pcd.points[p0], r)
                # n, idxs, ds = pcd_tree.search_hybrid_vector_3d(pcd.points[p0], r, max_nn)
                for i, p in enumerate(idxs):
                    if ds[i] > 0 and p not in points:
                        p1 = p
                        cur_line = [p0, p1]
                        cur_vec = pcd.points[cur_line[1]] - pcd.points[cur_line[0]]
                        cur_norm = np.linalg.norm(cur_vec)
                        cur_global_vec = pcd.points[cur_line[1]] - pcd.points[pi]
                        cur_global_norm = np.linalg.norm(cur_global_vec)
                        cur_r = cur_global_norm
                        if cur_r > max_r:
                            continue
                        if start_type > 0:  # First step
                            if len(local_lines) == 0:
                                g_dot = np.dot(cur_vec, global_vec)
                                ga = g_dot / (cur_norm * global_norm)
                                if ga < min_local_alpha:
                                    # print('local_alpha')
                                    continue
                                # Update
                                alphas.append(1.0)
                                betas.append(1.0)
                                normals.append([0, 0, 0])
                                # pcd.normals[p0] = [0, 0, 0]
                            else:  # Second+ step
                                prev_line = local_lines[-1]
                                prev_vec = pcd.points[prev_line[1]] - pcd.points[
                                    prev_line[0]]
                                prev_norm = np.linalg.norm(prev_vec)
                                # print(cur_vec, prev_vec)
                                # print(cur_norm, prev_norm)
                                ls_dot = np.dot(cur_vec, prev_vec)
                                a = ls_dot / (cur_norm * prev_norm)
                                if a < min_local_alpha:
                                    # print('local_alpha')
                                    continue
                                g_dot = np.dot(cur_global_vec, global_vec)
                                ga = g_dot / (cur_global_norm * global_norm)
                                if ga < min_global_alpha:
                                    # print('global_alpha')
                                    continue
                                n = np.cross(prev_vec, cur_vec)
                                n_norm = np.linalg.norm(n)
                                norm_n = n / n_norm if n_norm != 0 else n
                                if len(local_lines) > 1:  # Third+ step
                                    prev_n = normals[-2]
                                    prev_n_norm = np.linalg.norm(prev_n)
                                    ns_dot = np.dot(prev_n, n)
                                    if ns_dot != 0:
                                        b = ns_dot / (prev_n_norm * n_norm)
                                    else:
                                        b = 0
                                    if b < min_beta or b > max_beta:
                                        # print('local_beta')
                                        continue
                                else:
                                    b = 1.0
                                # Update
                                alphas.append(a)
                                normals.append(n)
                                # pcd.normals[p0] = norm_n
                                betas.append(b)
                        else:
                            if len(local_lines) > 0:
                                prev_line = local_lines[-1]
                                prev_vec = pcd.points[prev_line[1]] - pcd.points[
                                    prev_line[0]]
                                prev_norm = np.linalg.norm(prev_vec)
                                # print(cur_vec, prev_vec)
                                # print(cur_norm, prev_norm)
                                ls_dot = np.dot(cur_vec, prev_vec)
                                a = ls_dot / (cur_norm * prev_norm)
                                if a < min_local_alpha:
                                    # print('local_alpha')
                                    continue
                                g_dot = np.dot(cur_vec, global_vec)
                                ga = g_dot / (cur_norm * global_norm)
                                if ga < min_global_alpha:
                                    # print('global_alpha')
                                    continue
                                n = np.cross(prev_vec, cur_vec)
                                n_norm = np.linalg.norm(n)
                                norm_n = n / n_norm
                                # Update
                                alphas.append(a)
                                normals.append(n)
                                # pcd.normals[p0] = norm_n
                                prev_n = normals[-2]
                                prev_n_norm = np.linalg.norm(prev_n)
                                ns_dot = np.dot(prev_n, n)
                                b = ns_dot / (prev_n_norm * n_norm)
                                betas.append(b)
                            else:
                                g_dot = np.dot(cur_vec, global_vec)
                                ga = g_dot / (cur_norm * global_norm)
                                if ga < min_local_alpha:
                                    # print('local_alpha')
                                    continue
                                n = np.cross(global_vec, cur_vec)
                                n_norm = np.linalg.norm(n)
                                norm_n = n / n_norm
                                # Update
                                alphas.append(ga)
                                normals.append(n)
                                # pcd.normals[p0] = norm_n
                                betas.append(1.0)
                        # line color to alpha
                        a = alphas[-1]
                        # color = cmap((alphas[-1] + 1)/2)[:-1]
                        color = cmap(j / (len(dps) - 1))[:-1]
                        local_colors.append(color)
                        # point 0 color to beta
                        # b = betas[-1]
                        # p_color = cmap((betas[-1] + 1)/2)[:-1]
                        # pcd.colors[p0] = p_color
                        # print(a, b)
                        points.add(p)
                        local_lines.append(cur_line)
                        dists.append(ds[i])
                        rs.append(cur_r)
                        p0 = p1
                        break
            print('n_lines: {}'.format(len(local_lines)))
            if len(local_lines) > 2:
                lines.extend(local_lines)
                colors.extend(local_colors)
                # print(len(dists))
                # print(len(alphas))
                # print(len(betas))
                # print(alphas)
                # print(betas)
                xs = [sum(dists[:i]) for i, _ in enumerate(dists)]
                # plt.plot(xs, alphas, '.r-', xs, betas, '.g-')
                c = cmap(j / (len(dps) - 1))
                # plt.plot(xs, alphas, '.', color=c)
                # print(np.arccos(alphas))
                # print(dists)
                # print(np.arccos(alphas)/dists)
                betas_sign = np.sign(betas)
                # print(betas_sign)
                # print(np.arccos(alphas)*betas_sign)
                # kappa = np.arccos(alphas)*betas/dists
                kappa = np.arccos(alphas) * betas_sign / dists
                # kappa = np.arccos(alphas)/dists
                # kappa = np.arccos(alphas)
                # print(dists_center)
                global_rs.extend(rs)
                global_kappas.extend(kappa)
                plt.plot(rs, kappa, '.', color=c)
    # plt.ylim([-5, 5])
    plt.show()
    from matplotlib.image import NonUniformImage
    xedges = np.linspace(0, max(global_rs), 10)
    yedges = np.linspace(min(global_kappas), max(global_kappas), 10)
    H, xedges, yedges = np.histogram2d(
        global_rs, global_kappas,
        bins=(xedges, yedges),
        # density=True
    )
    H = H.T
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111, title='NonUniformImage: interpolated',
                         # aspect = 'equal',
                         xlim=xedges[[0, -1]],
                         ylim=yedges[[0, -1]])
    im = NonUniformImage(ax, interpolation='bilinear',
                         cmap=cm.get_cmap('viridis'))
    xcenters = (xedges[:-1] + xedges[1:]) / 2
    ycenters = (yedges[:-1] + yedges[1:]) / 2
    im.set_data(xcenters, ycenters, H)
    ax.images.append(im)
    plt.show()
    line_set = o3d.geometry.LineSet()
    line_set.points = pcd.points
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    key_to_callback = dict()
    key_to_callback[ord("5")] = change_background_to_black
    key_to_callback[ord("6")] = add_coord_axes
    o3d.visualization.draw_geometries_with_key_callbacks(
        [pcd, line_set], key_to_callback, width=500, height=500, left=800, top=50)
