import plane_extractor
import BPD
import numpy as np
import remove_wall
import open3d as o3d
import ransac as RS
import alpha_concave as AC
import remove_wall

#-----------------------------------------------------------------------------no_ransac + plane_extract + alpha_shape
# pc = o3d.io.read_point_cloud("cropped_livingroom.ply")

# plane, high_z, min_z = plane_extractor.extract_plane(pd, save_filename="cropped_plane_livingroom.txt")    # 다운샘플 + z축으로 25% 편집 투영

# boundary = np.loadtxt("cropped_plane_livingroom.txt") 
# result = AC.alpha_concave(boundary, "livingroom_boundary_a.txt")          #alpha_shape 처리
# #o3d.visualization.draw_geometries([result])



#------------------------------------------------------------------------------plane_seg + plane_extract + bpd
#pc = o3d.io.read_point_cloud("cropped_livingroom.ply")

# pd = RS.ransac(pc)            #plane_seg 처리
# plane, high_z, min_z = plane_extractor.extract_plane(pd, save_filename="cropped_plane_livingroom.txt")    # 다운샘플 + z축으로 25% 편집 투영
# boundary = BPD.cal_boundary(plane, save_filename="cropped_boundary_livingroom.txt")              # BPD, boundary point 도출 후, txt파일로 저장

# boundary = np.loadtxt("cropped_plane_livingroom.txt")
# pb = o3d.geometry.PointCloud()
# pb.points = o3d.utility.Vector3dVector(np.pad(np.array(boundary), (0, 1), 'constant', constant_values=0))
# o3d.visualization.draw_geometries([pb])



#---------------------------------------------------------------------------remove wall
# pc = o3d.io.read_point_cloud("cropped_livingroom.ply")
# plane, high_z, min_z = plane_extractor.extract_plane(pc)

# boundary = np.loadtxt("livingroom_boundary_a.txt") 

# removed = remove_wall.remove_wall(pc.points, boundary, method="cylinder", radius=0.053, min_z=min_z , high_z=high_z) # boundary point를 사용하여 벽 제거
# removed = remove_wall.remove_ceil_and_floor(removed, high_z - 0.15, min_z + 0.25) # 천장, 바닥 제거

# o3d.visualization.draw_geometries([removed])
