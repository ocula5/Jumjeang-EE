import numpy as np
import open3d as o3d

#points = np.loadtxt("cropped_2_obj.txt") 


def obj_removal_all(points):
    obje = points
    pb = o3d.geometry.PointCloud()
    pb.points = o3d.utility.Vector3dVector(obje)
    #o3d.visualization.draw_geometries([pb])
    
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pb.cluster_dbscan(eps=0.05, min_points=15, print_progress=True))
        
    max_label = labels.max()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    pb.colors = o3d.utility.Vector3dVector(colors[:, :3])
    
    labeling_point = []
    for i in range(max_label):
        label_n = []
        for j in range(len(obje)) :
            if labels[j] == i:
                label_n.append(j)
        labeling_point.append(label_n)
    
    
    scale_space = []
    for i in range(max_label):
        space = []
        for j in range(max_label) :
            space = pb.select_by_index(labeling_point[i])
        scale_space.append(space) 
    
    final_obj = []
    for i in range(max_label):
        if scale_space[i].get_axis_aligned_bounding_box().volume() > 0.02:
            arr = np.asarray(scale_space[i].points)
            for j in range(len(arr)):
                final_obj.append(arr[j])
                
    result = np.array(final_obj)

    # pf = o3d.geometry.PointCloud()
    # pf.points = o3d.utility.Vector3dVector(result)
    # o3d.visualization.draw_geometries([pf])
    
    return result
    
    
