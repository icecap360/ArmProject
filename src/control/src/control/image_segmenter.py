#! /usr/bin/env python

import rospy
from control.msg import class_list, arm_parameters
from control.srv import isGo
import ros_numpy
import numpy as np 
from sensor_msgs.msg import PointCloud2


class imageSegmenter:
    def __init__(self):
        self.sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.pointCloudToXYZRGB)
        self.execute =True

    def pointCloudToXYZRGB(self, ros_cloud):
        if not self.execute:
            return
        ros_cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(ros_cloud)
        xyz = ros_numpy.point_cloud2.get_xyz_points(ros_cloud_arr, remove_nans=False)
        rgb = ros_numpy.point_cloud2.split_rgb_field(ros_cloud_arr)
        #print(xyz.shape)
        r = rgb['r']
        g = rgb['g']
        b=rgb['b']
        img = np.array([r,g,b]) #shape is 3,480,640
        img = np.moveaxis(img, 0,2) #shape is 480,640,3
        self.img = img
        print(self.img.shape)
        self.execute = False


if __name__ == '__main__':
    rospy.init_node('imageSegmenter', anonymous=True)
    # call constructor
    image_segmenter = imageSegmenter()

    
    # main loop
    rospy.spin()
