#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import ros_numpy
import numpy as np
from sensor_msgs.msg import Image
import numpy.lib.recfunctions as rf


class pointCloudPCL:
    def __init__(self):
        self.image_pub = rospy.Publisher('arm_vision_image', Image, queue_size=10)
    def pointCloudToXYZRGB(self, ros_cloud):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB

            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message

            Returns:
                xyzrgb numpy array
        """
        ros_cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(ros_cloud)
        xyz = ros_numpy.point_cloud2.get_xyz_points(ros_cloud_arr, remove_nans=False)
        rgb = ros_numpy.point_cloud2.split_rgb_field(ros_cloud_arr)
        print(xyz.shape)
        r = rgb['r']
        g = rgb['g']
        b=rgb['b']
        img = np.array([r,g,b]) #shape is 3,480,640
        img = np.moveaxis(img, 0,2) #shape is 480,640,3
        #better alternative: rf.structured_to_unstructured(rgb), look into it
        self.image_pub.publish(
            ros_numpy.image.numpy_to_image(
                img, "rgb8"))

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    point_coud_pcl = pointCloudPCL()
    rospy.Subscriber('/camera/depth/points', PointCloud2, point_coud_pcl.pointCloudToXYZRGB)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
