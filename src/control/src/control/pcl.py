#!/usr/bin/env python
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np

def callback(data):
    print('yeah')
    pc = ros_numpy.numpify(data)
    print(pc.shape[0])
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    p = pcl.PointCloud(np.array(points, dtype=np.float32))


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/camera/depth/points', PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

"""
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy

def callback(data):
    print('yeah')
    #pc = ros_numpy.numpify(data)
    #points=np.zeros((pc.shape[0],3))
    #points[:,0]=pc['x']
    #points[:,1]=pc['y']
    #points[:,2]=pc['z']
    #p = pcl.PointCloud(np.array(points, dtype=np.float32))
print('here')
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("camera/depth/points", PointCloud2, callback)
print('subscribed')
rospy.spin()"""