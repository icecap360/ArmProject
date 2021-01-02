#! /usr/bin/env python

import rospy
from control.msg import object_dimension
from control.srv import calculateDim

class calculateDimension:
    def __init__(self):
        # also subsribe to image and pointcloud2 topics
        self.dimension_topic = rospy.Publisher('object_dimension', object_dimension, queue_size=10)
        self.serv = rospy.Service('calculate_dimension', calculateDim, self.calculate_dimension)
        self.length = 0
        self.width = 0
        self.height = 0

    def get_length(self):
        return self.length
    def get_width(self):
        return self.width
    def get_height(self):
        return self.height
    def set_length(self, length):
        self.length = length
    def set_width(self, width):
        self.width = width
    def set_height(self, height):
        self.height = height

    # service
    def calculate_dimension(self, req):
        self.set_length(5)
        self.set_width(5)
        self.set_height(5)
        return True

if __name__ == '__main__':
    rospy.init_node('calculate_dimension')
    # call constructor
    calculate_dim = calculateDimension()
    # main loop
    r = rospy.Rate(10) #this should be decreased, arm_params is quite big
    while not rospy.is_shutdown():
        calculate_dim.dimension_topic.publish(
            calculate_dim.get_length(), calculate_dim.get_width(),
            calculate_dim.get_height() )
        r.sleep();
