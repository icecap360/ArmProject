#!/usr/bin/env python

import rospy
from control.srv import (
    isObjectPicked
)

class hasObject:
	def __init__(self):
		self.update_has_object_serv = rospy.Service('update_has_object', isObjectPicked, self.update_has_object)
	
	def update_has_object(self, req):
		self.has_object = False
		return self.has_object

if __name__ == '__main__':
    rospy.init_node('has_object', anonymous=True)
    # call constructor
    has_object = hasObject()
    # main loop
    rospy.spin()
