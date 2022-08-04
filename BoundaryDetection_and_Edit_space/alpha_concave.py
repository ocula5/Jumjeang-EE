import os
import sys
import pandas as pd
import numpy as np
from descartes import PolygonPatch
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(os.getcwd()))
import alphashape
import open3d as o3d
from shapely.geometry import Polygon


def alpha_concave(points, save_filename):
    boundary = alphashape.alphashape(points, 10)
    x = []
    y = []
    some_poly = boundary

    # Extract the point values that define the perimeter of the polygon
    x, y = some_poly.exterior.coords.xy
    out = [None] * (len(x)-1)
    for i in range(0, len(x)-1):
        out[i]=[x[i], y[i]]

    result = np.array(out)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.pad(np.array(result), (0, 1), 'constant', constant_values=0))
    
    points = np.array(pcd.points)

    if save_filename != None:
        np.savetxt(save_filename, points[:, :2])
        
    return pcd   
    #o3d.visualization.draw_geometries([pcd])

