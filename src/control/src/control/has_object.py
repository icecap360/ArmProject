#!/usr/bin/env python

import rospy
from control.srv import (
    isObjectPicked
)

class hasObject:
	def __init__(self):
		self.has_object_serv = rospy.Service('is_object_picked', isObjectPicked, self.has_object)
	def has_object(self):
		return True 

if __name__ == '__main__':
    rospy.init_node('has_object', anonymous=True)
    # call constructor
    has_object = hasObject()
    # main loop
    rospy.spin()
