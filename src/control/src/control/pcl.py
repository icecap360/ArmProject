#!/usr/bin/env python
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np

def callback(data):
    print('yeah')
    pc = pcl.ros_numpy.numpify(data)
    print(pc.shape)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']
    p = pcl.PointCloud(np.array(points, dtype=np.float32))

def callback(ros_cloud):
    """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

        Args:
            ros_cloud (PointCloud2): ROS PointCloud2 message

        Returns:
            pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
    """
    points_list = []
    
    print(ros_numpy.point_cloud2.split_rgb_field(ros_cloud) )
    print(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(ros_cloud))
    
    
    #print('shape', pcl.ros_numpy.numpify(ros_cloud))#[0][0])
    for i in rawPoints:
        for j in i:
            pass#print(type(j))
        #print()#type(i))
    rawPoints = pc2.read_points(ros_cloud, skip_nans=True)

    #for data in :
    #    points_list.append([data[0], data[1], data[2], data[3]])
    #pcl_data = pcl.PointCloud_PointXYZRGB()
    #pcl_data.from_list(points_list)

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