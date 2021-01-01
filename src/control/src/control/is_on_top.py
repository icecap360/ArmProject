#! /usr/bin/env python

import rospy
import actionlib
import control.msg
from control.msg import ensureIsOnTopFeedback, ensureIsOnTopResult, ensureIsOnTopAction

class ensure_is_on_top(object):
	# create messages that are used to publish feedback/result
	_feedback = ensureIsOnTopFeedback()
	_result = ensureIsOnTopResult()
	_action = ensureIsOnTopAction
	def __init__(self):
		self._action_name = 'ensure_is_on_top'
		self._as = actionlib.SimpleActionServer(self._action_name, self._action, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()
		#error tolerance should be a programmable parameter from paramserver
		self.error_tolerance = 100 
	
	def calculate_error(self):
		error = 1.5
		pose = 0.5
		return error,pose

	def execute_cb(self, goal):
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
			elif (error < self.error_tolerance):
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
	server = ensure_is_on_top()
	rospy.spin()