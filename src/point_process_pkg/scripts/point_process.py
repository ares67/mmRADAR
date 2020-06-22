#!/usr/bin/env python3
# coding: utf-8

import argparse
import rospy
import numpy as np
import math
import struct
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from sklearn.cluster import DBSCAN

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker
from std_msgs.msg import Header
# from sort import Sort

debug = 1   # debug=1时用matplotlib作图


class data_process:
    def __init__(self, duration):
        self.duration = duration
        self.doppler_bin_num = 2*int(round(rospy.get_param("/mmWave_Manager/numLoops")))    # num of doppler bins ~ max_vel/vel_res
        self.frame_id = 'base_radar_link'
        self.radar_vel_max = rospy.get_param("/mmWave_Manager/max_doppler_vel")
        self.radar_vel_res = rospy.get_param("/mmWave_Manager/doppler_vel_resolution")
        self.pointcloud_all =  np.empty((0,6))
        self.num = []
        self.frame_count = 0
        self.ax = plt.figure().add_subplot(1,1,1)
        # self.mot_tracker = Sort(args.sort_max_age,args.sort_min_hit) 
        plt.ion()


    def point_process(self, data):
        assert isinstance(data, PointCloud2)

        # pointcloud is from one frame
        pointcloud = np.array(list(point_cloud2.read_points(
            data, skip_nans=True, field_names = ("x", "y", "z","range","velocity", "intensity")))) 
        # pointcloud_all accumulates pointcloud of self.duration frames
        self.pointcloud_all = np.append(self.pointcloud_all, pointcloud, axis=0)
        self.num.append(np.size(pointcloud,0))  # self.num is a queue to record #(points) of each frame
        self.frame_count += 1
        if self.frame_count > self.duration:
            self.pointcloud_all = self.pointcloud_all[self.num[0]:, :]
            self.num = self.num[1:]
            self.clustering()

    def clustering(self):
        
        weight = np.array([[1, 0, 0], [0, 3, 0], [0, 0, 1]])
        y_pred = DBSCAN(eps = 0.5, min_samples = 50).fit_predict( np.dot(self.pointcloud_all[:, [0, 1, 4]], weight) )
        #y_pred = DBSCAN(eps = 1.5, min_samples = 30).fit_predict(self.pointcloud_all[:, [0 ,1 ,2]]) 
        pointcloud_clustered = np.append(self.pointcloud_all, np.atleast_2d(y_pred).T, axis=1)
        pointcloud_filtered = np.array([x for x in pointcloud_clustered if x[6] != -1 ])    # class -1 for noise
        # pointcloud_sorted = sorted(pointcloud_filtered, key=lambda x:x[6])
        
        # generate bounding boxes
        rect = []   # min_x, max_x, min_y, max_y
        num_cluster = int(max(pointcloud_filtered[:, 6])) + 1
        for i in range(num_cluster):
            points = np.array([x for x in pointcloud_filtered if x[6] == i])
            rect.append([min(points[:, 0]), max(points[:, 0]), min(points[:, 1]), max(points[:, 1])])
        print(np.array(rect), num_cluster)

        # 打印噪音点的数量
        #print(np.shape(pointcloud_clustered), np.shape(pointcloud_filtered))

        if debug == 1:
            self.drawing(pointcloud_clustered[:, [0, 1, 5]], rect)
        else:
            self.pub1_.publish(self.array_to_pointcloud2(pointcloud_filtered))
            # self.pub2_.publish(self.generate_bbox(pointcloud_filtered))
        
    

    # plt.cla()以及相邻帧之间的剧烈变化导致画面有抖动
    def drawing(self, point, rect):
        plt.pause(0.01)
        plt.cla() 
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlabel('Z')
        self.ax.set_ylabel('X ')
    #edited
        #self.ax.scatter(point[:, 0], point[:, 1], c = point[:, 2], marker = '.')
        self.ax.scatter(point[:, 0], point[:, 1], c = point[:, 2], marker = '.')
        for x in rect:
            self.ax.add_patch( plt.Rectangle((x[0], x[2]), x[1]-x[0], x[3]-x[2], linewidth=1, edgecolor='r',fill = False) )


    def array_to_pointcloud2(self, data):
        msg =  PointCloud2()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = data.shape[0]

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('range', 12, PointField.FLOAT32, 1),
            PointField('velocity', 16, PointField.FLOAT32, 1),
            PointField('intesity', 20, PointField.FLOAT32, 1),
            PointField('class', 24, PointField.FLOAT32, 1),
        ]
        msg.is_bigendian = 0
        msg.is_dense = 1
        msg.point_step = 28
        msg.row_step = msg.point_step * data.shape[0]
        msg.data = np.asarray(data, np.float32).tostring()
        return msg 
            

    def main(self):
        rospy.init_node('clustering')
        self.sub_ = rospy.Subscriber('/mmWaveDataHdl/RScan', PointCloud2, self.point_process, queue_size=10)
        self.pub1_ = rospy.Publisher('radar/points', PointCloud2, queue_size=10)
        self.pub2_ = rospy.Publisher('radar/rects', Marker, queue_size=10)

        if debug == 1:
            plt.show(block = True)    # 需要用matplotlib作图时，用此语句代替rospy.spin()，程序会停在此处
        else:
            rospy.spin() 


if __name__ == '__main__':
    fps =  1000/rospy.get_param("/mmWave_Manager/framePeriodicity")
    print(fps)     # f_UART = f_mmWaveDataHdl/RScan = f_CallbackFunction = fps
    decay_time = 0.3
    duration = round(fps * decay_time)    
    data_process(duration).main()
