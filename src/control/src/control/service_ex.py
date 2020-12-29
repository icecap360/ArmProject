#!/usr/bin/env python

import rospy
from control.srv import AddTwoInts, AddTwoIntsResponse

def add_two_ints(req):
  return AddTwoIntsResponse(req.a + req.b)

rospy.init_node('add_two_ints_server')
s = rospy.Service('add_two_ints', AddTwoInts, add_two_ints)
rospy.spin()
