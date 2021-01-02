#! /usr/bin/env python

import rospy
import actionlib

from params import get_lateral_error_tolerance
from control.msg import (
	ensureIsOnTopFeedback, 
	ensureIsOnTopResult, 
	ensureIsOnTopAction,
	arm_parameters
)

class ensureIsOnTop(object):
	# create messages that are used to publish feedback/result
	_feedback = ensureIsOnTopFeedback()
	_result = ensureIsOnTopResult()
	_action = ensureIsOnTopAction
	def __init__(self):
		self._action_name = 'ensure_is_on_top'
		self._as = actionlib.SimpleActionServer(self._action_name, self._action, ensure_is_on_top_cb=self.ensure_is_on_top_cb, auto_start = False)
		self._as.start()
		#error tolerance should be a programmable parameter from paramserver
		self.lateral_error_tolerance = get_lateral_error_tolerance()
	
	def calculate_error(self):
		error = 1.5
		pose = 0.5
		return error,pose

	def ensure_is_on_top_cb(self, goal):
		# helper variables
		r = rospy.Rate(1)
		success = True
		
		# calculate the current error
		error = 99999#this should be infinite
		while (True):
			error, next_pose = self.calculate_error() 
			self._feedback.current_error = error
			self._as.publish_feedback(self._feedback)
			# publish info to the console for the user
			print('The current error is %f'% error)
			if self._as.is_preempt_requested():
				print('Preempted Arm is on top of object verification')
				self._as.set_preempted()
				success = False
				break
			elif (error < self.lateral_error_tolerance):
				print('The error is within the tolerance')
				r.sleep()
				break
			else:
				#move robot to next_pose, import helpers from moveit
				pass
		if success:
			self._result.is_on_top = True
			print('Is on top verification succeded')
			self._as.set_succeeded(self._result)

if __name__ == '__main__':
	rospy.init_node('is_on_top')
	ensure_is_on_top = ensureIsOnTop()
	rospy.spin()