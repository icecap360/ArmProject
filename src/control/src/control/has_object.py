#!/usr/bin/env python

import rospy
from control.srv import (
    doService, 
    isObjectPicked
)

class hasObject:
    def __init__(self):
        self.update_has_object_serv = rospy.Service('update_has_object', doService, self.update_has_object)
        self.has_object_serv = rospy.Service('has_object', isObjectPicked, self.get_has_object)
        self.has_object = False

    def get_has_object(self, req=None):
        return self.has_object
    def set_has_object(self, has_object):
        self.has_object = has_object

    """ service """
    def update_has_object(self, req):
		self.set_has_object(False)
		return True

if __name__ == '__main__':
    rospy.init_node('has_object', anonymous=True)
    # call constructor
    has_object = hasObject()
    # main loop
    rospy.spin()
