import numpy as np
import open3d as o3d
import plane_extractor

# Input data type ----------------------------------------------
# points : <class 'numpy.ndarray'>
def make_wall(boundary_points, min_z, high_z, sample_rate=0.02) :
    big = max(min_z, high_z)
    small = min(min_z, high_z)

    result = np.zeros((1,3))

    for i in np.arange(small, big, sample_rate) :
        temp = np.pad(np.array(boundary_points), [(0,0),(0, 1)], 'constant', constant_values=i)
        result = np.concatenate((result, temp), axis=0)

    # return type is <class 'numpy.ndarray'>
    return result



# Input data type ----------------------------------------------
# points : <class 'numpy.ndarray'>
def remove_boundary_noise(points):
    pb = o3d.geometry.PointCloud()
    pb.points = o3d.utility.Vector3dVector(np.pad(np.array(points), (0, 1), 'constant', constant_values=0))
    cl, ind = pb.remove_statistical_outlier(nb_neighbors=3,
                                                    std_ratio=2.0)

    inlier_cloud = pb.select_by_index(ind)
    return inlier_cloud
    # return type is <class 'open3d.cpu.pybind.geometry.PointCloud'>



# Input data type ----------------------------------------------
# point_cloud : <class 'open3d.cpu.pybind.geometry.PointCloud'>
# boundary : <class 'numpy.ndarray'>
def generate_wall_point(point_cloud, boundary, save_filename=None):
    plane, high_z, min_z = plane_extractor.extract_ceil(point_cloud)
    improve_boundary = remove_boundary_noise(boundary)

    pre_arr = np.array(improve_boundary.points)

    my_arr = []
    for i in range(len(pre_arr)):
        arr_n = []
        for j in range(len(pre_arr[i])-1) :
            arr_n.append(pre_arr[i][j])
        my_arr.append(arr_n)

    big = max(min_z, high_z)
    small = min(min_z, high_z)

    wall = make_wall(my_arr, min_z, high_z)

    if save_filename != None:
        np.savetxt(save_filename, wall[1:])

    pf = o3d.geometry.PointCloud()
    pf.points = o3d.utility.Vector3dVector(wall[1:])
    o3d.visualization.draw_geometries([pf])
    return wall[1:]




# *test*
# pc = o3d.io.read_point_cloud("cropped_1.ply")
# boundary = np.loadtxt("cropped_1_alpha.txt")
# result = generate_wall_point(pc, boundary, save_filename="wall_test2.txt")
