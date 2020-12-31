#!/usr/bin/env python

import rospy
from control.srv import isFieldAnalyzed, isFieldAnalyzedResponse

def analyze():
	print('Searching field for what objects are present and where in the field')
	return True

rospy.init_node('classifier')
s = rospy.Service('is_field_analyzed', isFieldAnalyzed, analyze)
rospy.spin()

