from abc import ABCMeta, abstractmethod
import rospy
from control.srv import isGo, isFieldAnalyzed, isMoveComplete, setTaskComplete
import time

"""HELPERS"""
def fail_error(mssg):
	fail = 'FAIL:'+mssg+', ABORTING'
	print(fail)
	rospy.signal_shutdown(fail)


"""SERVICES DECLARATION AND INITIALIZATION"""
class SERVICES:
	#Must initialize services object (in main) before any of the
	#transitions (which depend on the services) are called.
	def initialize(self):
		rospy.wait_for_service('is_go')
		self.is_go = rospy.ServiceProxy('is_go', isGo)
		rospy.wait_for_service('is_field_analyzed')
		self.is_field_analyzed = rospy.ServiceProxy('is_field_analyzed', isFieldAnalyzed)
		# change
		rospy.wait_for_service('is_field_analyzed')
		self.lateral_move = rospy.ServiceProxy('lateral_move', isMoveComplete)
		rospy.wait_for_service('set_task_complete')
		self.set_task_complete = rospy.ServiceProxy('set_task_complete', setTaskComplete)
		self.initialize_complete()
	def initialize_complete(self):
		print('All services Setup')
	# Defining all services getters
	def call_is_go(self):
		return self.is_go()
	def call_is_field_analyzed(self):
		return self.is_field_analyzed()
	def call_lateral_move(self):
		return self.lateral_move()
	def call_set_task_complete(self):
		return self.set_task_complete()
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
		print('Holding robot constant')
	def exit(self):
		print('Go signaled, starting')
neutral_pose = NEUTRAL_POSE()

class LOCATE_OBJECT(abstract_state):
	def entry(self):
		print('Analysing the current field of objects')
		success = services.call_is_field_analyzed().is_field_analyzed
		if not success:
			fail_msg = 'The arm was unable analyze the field'
			fail_error(fail_msg)
	def exit(self):
		print('Analysis done')
locate_object = LOCATE_OBJECT()

class SET_DESIRED_OBJECT(abstract_state):
	def entry(self):
		print('Choosing the next desired object')
		success = services.call_set_task_complete().is_complete
		if not success:
			fail_msg = 'No tasks left to complete'
			fail_error(fail_msg)
set_desired_object = SET_DESIRED_OBJECT()

class LATERAL_MOVE(abstract_state):
	def entry(self):
		print('Starting lateral move')
		success = services.call_lateral_move().is_move_complete
		if not success:
			fail_msg = 'The arm was unable to move laterally to the top of the desired object'
			fail_error(fail_msg)
lateral_move = LATERAL_MOVE()

""""TRANSITION DEFINITIONS"""

class is_go(abstract_transition):
	def condition(self):
		return services.call_is_go()
class default(abstract_transition):
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
	locate_object : [default(set_desired_object)],
	set_desired_object : [default(lateral_move)],
	lateral_move : [default(neutral_pose)]
}
