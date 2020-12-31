from abc import ABCMeta, abstractmethod 
import rospy
from control.srv import isGo

import time
class SERVICES:
	def initialize(self):
		rospy.wait_for_service('is_go')
		self.is_go = rospy.ServiceProxy('is_go', isGo)
		self.initialize_complete()
	def initialize_complete(self):
		print('All services Setup')
	def call_is_go(self):
		return self.is_go()
services = SERVICES()

"""ABSTRACT STATES AND TRANSITIONS"""
class abstract_state:
	__metaclass__ = ABCMeta
	@abstractmethod
	def entry(self):
		pass
class abstract_transition:
	__metaclass__ = ABCMeta
	def __init__(self, next_state):
		self.next_state = next_state
	@abstractmethod
	def condition(self):
		pass

""""STATES DEFINITION"""
class NEUTRAL_POSE(abstract_state):
	def entry(self):
		print('Arm Ready')
	def during(self):
		print('Waiting for user GO signal')
	def exit(self):
		print('Go signal Received')
neutral_pose = NEUTRAL_POSE()

class LOCATE_OBJECT(abstract_state):
	def entry(self):
		print('Analysing the current field of objects')
	def exit(self):
		print('Analyses done')
locate_object = LOCATE_OBJECT()

class SET_DESIRED_OBJECT(abstract_state):
	def entry(self):
		print('Choosing the next desired object')
set_desired_object = SET_DESIRED_OBJECT()

""""TRANSITION DEFINITIONS"""
	
class is_go(abstract_transition):
	def condition(self):
		return services.call_is_go()
class transition2(abstract_transition):
	def condition(self):
		return True
class transition3(abstract_transition):
	def condition(self):
		return True
	
"""THE LOGIC OF THE STATEFLOW STARTS HERE
THE FINITE STATE MACHINE IS AN ADJACENCY LIST.
"""
finite_state_machine = {
	neutral_pose : [is_go(locate_object)],
	locate_object : [transition2(set_desired_object)],
	set_desired_object : [transition3(neutral_pose)],
}
