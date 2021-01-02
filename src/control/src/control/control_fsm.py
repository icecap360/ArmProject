from abc import ABCMeta, abstractmethod
import rospy
import actionlib
from control.msg import (
	ensureIsOnTopGoal,
	ensureIsOnTopAction
)
from control.srv import (
	isGo,
	locateAllObjects,
	isMoveComplete,
	setTaskComplete,
	setObject
)
import time

"""HELPERS"""
def fail_error(mssg):
	fail = 'FAIL:'+mssg+', ABORTING'
	print(fail)
	rospy.signal_shutdown(fail)
def has_exit_commands(currentState):
	return hasattr(currentState, 'exit')
def has_during_commands(currentState):
	return hasattr(currentState, 'during')

"""SERVICES DECLARATION AND INITIALIZATION"""
class SERVICES:
	#Must initialize services object (in main) before any of the
	#transitions (which depend on the services) are called.
	def initialize(self):
		rospy.wait_for_service('is_go')
		self.is_go = rospy.ServiceProxy('is_go', isGo)
		rospy.wait_for_service('locate_all_objects')
		self.locate_all_objects = rospy.ServiceProxy('locate_all_objects', locateAllObjects)
		rospy.wait_for_service('lateral_move')
		self.lateral_move = rospy.ServiceProxy('lateral_move', isMoveComplete)
		rospy.wait_for_service('set_task_complete')
		self.set_task_complete = rospy.ServiceProxy('set_task_complete', setTaskComplete)
		rospy.wait_for_service('set_object')
		self.set_object = rospy.ServiceProxy('set_object', setObject)
		self.ensure_is_on_top = actionlib.SimpleActionClient('ensure_is_on_top', ensureIsOnTopAction)
		self.ensure_is_on_top.wait_for_server()
		self.initialize_complete()
	def initialize_complete(self):
		print('All services Setup')
	# Defining all services getters
	def call_is_go(self):
		return self.is_go()
	def call_locate_all_objects(self):
		return self.locate_all_objects()
	def call_lateral_move(self):
		return self.lateral_move()
	def call_set_task_complete(self):
		return self.set_task_complete()
	def call_set_object(self):
		return self.set_object()
	def call_ensure_is_on_top(self):
		goal = ensureIsOnTopGoal()
		services.ensure_is_on_top.send_goal(goal)
		services.ensure_is_on_top.wait_for_result()
		#no point of result, isontop keeps running unless preempted or error<tolerance
		return services.ensure_is_on_top.get_result().is_on_top
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

class LOCATE_ALL_OBJECT(abstract_state):
	def entry(self):
		print('Analysing the current field of objects')
		success = services.call_locate_all_objects().is_field_analyzed
		if not success:
			fail_msg = 'The arm was unable to analyze the field'
			fail_error(fail_msg)
	def exit(self):
		print('Analysis done')
locate_all_object = LOCATE_ALL_OBJECT()

class SET_DESIRED_OBJECT(abstract_state):
	def entry(self):
		# change condition for running
		if True:
			print('Setting object as complete')
			task_resp = services.call_set_task_complete()
			if not task_resp.success:
				fail_msg = 'The arm could not set the task as complete'
				fail_error(fail_msg)
			else:
				if task_resp.is_empty:
					print('No objects left')
					# add stuff here
		print('Choosing the next desired object')
		success = services.call_set_object().success
		if not success:
			fail_msg = 'The arm was unable to set the desired object'
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

class ENSURE_IS_ON_TOP(abstract_state):
	def entry(self):
		print('Verifying arm is on top of object')
		services.call_ensure_is_on_top()
	def exit(self):
		print('Arm is on top of object')
ensure_is_on_top = ENSURE_IS_ON_TOP()

class CALCULATE_DIMENSIONS(abstract_state):
	def entry(self):
		print('Calculating dimensions of object')
calculate_dimensions = CALCULATE_DIMENSIONS()

""""TRANSITION DEFINITIONS"""

class is_go(abstract_transition):
	def condition(self):
		return services.call_is_go().is_go
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
	neutral_pose : [is_go(locate_all_object)],
	locate_all_object : [default(set_desired_object)],
	set_desired_object : [default(lateral_move)],
	lateral_move : [default(ensure_is_on_top)],
	ensure_is_on_top : [default(calculate_dimensions)],
	calculate_dimensions : [default(neutral_pose)]
}
