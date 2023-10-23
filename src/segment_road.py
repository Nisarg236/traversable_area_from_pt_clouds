#!/usr/bin/env python3
import math
import numpy as np
import pcl
from functions_used import *

def calc_angle(plane, axis):
    if axis == "x":
        I = np.array([1,0,0])
    elif axis == "y":
        I = np.array([0,1,0])
    elif axis == "z":
        I = np.array([0,0,1])
    n = np.array([plane[0], plane[1], plane[2]])
    angle = math.asin(math.sin(np.dot(I, n) / math.sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]))) * 180 / math.pi
    return angle

def extract_points_from_pcd_file(pcd_file):
    cloud = pcl.load(pcd_file)
    points = np.array(cloud)
    x, y, z = points[:,0], points[:,1], points[:,2]
    return x, y, z

def save_points_to_pcd_file(points, filename):
    point_cloud = pcl.PointCloud()
    point_cloud.from_array(points.astype(np.float32))
    pcl.save(point_cloud, filename)

def extract_road_and_curb_points(x, y, z):
    points = np.vstack((x, y, z)).transpose()
    planes = DetectMultiPlanes(points, min_ratio=0.70, threshold=0.01, iterations=90)
    planes_eqn = []
    points_plane = []
    intercept = []
    i = 0
    while i < len(planes):
        angle_x = calc_angle(planes[i][0], 'x')
        angle_y = calc_angle(planes[i][0], 'y')
        print('anglex: , angley: ', angle_x, angle_y)
        if angle_x <= 60 and angle_x >= -60 and angle_y <= 60 and angle_y >= -60:
            intercept_z = planes[i][0][3] / planes[i][0][2]
            intercept.append(intercept_z)
            planes_eqn.append(planes[i][0])
            points_plane.append(planes[i][1])
        i += 1
    print(len(planes_eqn))
    curb_index = np.argmax(intercept)
    curb_points = points_plane[curb_index]

    road_index = np.argmin(intercept)
    road_points = points_plane[road_index]

    index = 0
    for i in intercept:
        if index != road_index and index != curb_index:
            curb_points = np.row_stack((curb_points, points_plane[index]))
        index += 1

    return road_points, curb_points

def main():
    pcd_file = '/home/ott-testing/catkin_ws/src/pcdWindows/processed1.pcd'
    output_dir = './'
    x, y, z = extract_points_from_pcd_file(pcd_file)
    road_points, curb_points = extract_road_and_curb_points(x, y, z)
    save_points_to_pcd_file(road_points, output_dir + 'road.pcd')
    save_points_to_pcd_file(curb_points, output_dir + 'curb.pcd')

if __name__ == "__main__":
    main()
