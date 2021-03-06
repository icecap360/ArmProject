from abc import ABCMeta, abstractmethod
import rospy
import actionlib
from control.msg import (
	ensureIsOnTopGoal,
	ensureIsOnTopAction
)
from control.srv import (
	isGo,
	doService,
	isTaskEmpty,
	isObjectPicked,
)
#import time

"""HELPERS"""
def process_do_service(success, fail_msg, succ_msg=None):
	#success = services.call_move_to_neutral_pose()
	if not success:
		fail_error(fail_msg)
	elif succ_msg != None:
		print(succ_msg)
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
		rospy.wait_for_service('move_to_neutral_pose')
		self.move_to_neutral_pose = rospy.ServiceProxy('move_to_neutral_pose', doService)
		rospy.wait_for_service('is_go')
		self.is_go = rospy.ServiceProxy('is_go', isGo)
		rospy.wait_for_service('locate_all_objects')
		self.locate_all_objects = rospy.ServiceProxy('locate_all_objects', doService)
		rospy.wait_for_service('lateral_move')
		self.lateral_move = rospy.ServiceProxy('lateral_move', doService)
		rospy.wait_for_service('set_task_complete')
		self.set_task_complete = rospy.ServiceProxy('set_task_complete', doService)
		rospy.wait_for_service('is_task_empty')
		self.is_task_empty = rospy.ServiceProxy('is_task_empty', isTaskEmpty)
		rospy.wait_for_service('set_object')
		self.set_object = rospy.ServiceProxy('set_object', doService)
		self.ensure_is_on_top = actionlib.SimpleActionClient('ensure_is_on_top', ensureIsOnTopAction)
		self.ensure_is_on_top.wait_for_server()
		rospy.wait_for_service('calculate_dimension')
		self.calculate_dimension = rospy.ServiceProxy('calculate_dimension', doService)
		rospy.wait_for_service('pick_object')
		self.pick_object = rospy.ServiceProxy('pick_object', doService)
		rospy.wait_for_service('update_has_object')
		self.update_has_object = rospy.ServiceProxy('update_has_object', doService)
		rospy.wait_for_service('has_object')
		self.has_object = rospy.ServiceProxy('has_object', isObjectPicked)
		rospy.wait_for_service('restart_pick_object')
		self.restart_pick_object = rospy.ServiceProxy('restart_pick_object', doService)
		rospy.wait_for_service('place_object')
		self.place_object = rospy.ServiceProxy('place_object', doService)
		self.initialize_complete()
	def initialize_complete(self):
		print('All services Setup')
	# Defining all services getters
	def call_move_to_neutral_pose(self):
		return self.move_to_neutral_pose().success
	def call_is_go(self):
		return self.is_go().is_go
	def call_locate_all_objects(self):
		return self.locate_all_objects().success
	def call_lateral_move(self):
		return self.lateral_move().success
	#
	def call_set_task_complete(self):
		return self.set_task_complete().success
	def call_is_task_empty(self):
		return self.is_task_empty().is_empty
	def call_not_is_task_empty(self):
		return not self.call_is_task_empty()

	def call_set_object(self):
		return self.set_object().success
	def call_ensure_is_on_top(self):
		goal = ensureIsOnTopGoal()
		self.ensure_is_on_top.send_goal(goal)
		self.ensure_is_on_top.wait_for_result()
		#no point of result, isontop keeps running unless preempted or error<tolerance
		return self.ensure_is_on_top.get_result().is_on_top
	def call_pick_object(self):
		return self.pick_object().success
	def call_update_has_object(self):
		return self.update_has_object().success
	def call_calculate_dimension(self):
		return self.calculate_dimension().success
	def call_has_object(self):
		return self.has_object().is_object_picked
	def call_not_has_object(self):
		return not self.call_has_object()
	def call_restart_pick_object(self):
		return self.restart_pick_object().success
	def call_place_object(self):
		return self.place_object().success
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
		print('Moving arm to neutral pose')
		process_do_service(
			services.call_move_to_neutral_pose(),
			'The arm was unable to move to neutral pose',
			'Arm Ready')
	def during(self):
		print('Holding robot constant')
	def exit(self):
		print('Go signaled, starting')
neutral_pose = NEUTRAL_POSE()

class LOCATE_ALL_OBJECT(abstract_state):
	def entry(self):
		print('Analysing the current field of objects')
		process_do_service(
			services.call_locate_all_objects(),
			'The arm was unable to analyze the field',
			'Analysis done')
locate_all_object = LOCATE_ALL_OBJECT()

class SET_DESIRED_OBJECT(abstract_state):
	def entry(self):
		# change condition for running
		# if True:
		# 	print('Setting object as complete')
		# 	task_resp = services.call_set_task_complete()
		# 	if not task_resp.success:
		# 		fail_msg = 'The arm could not set the task as complete'
		# 		fail_error(fail_msg)
		# 	else:
		# 		if task_resp.is_empty:
		# 			print('No objects left')
		# 			# add stuff here
		print('Updating the desired object')
		success = services.call_set_object()
		if not success:
			fail_msg = 'The arm was unable to set the desired object'
			fail_error(fail_msg)
set_desired_object = SET_DESIRED_OBJECT()

class LATERAL_MOVE(abstract_state):
	def entry(self):
		print('Starting lateral move')
		process_do_service(
			services.call_lateral_move(),
			'The arm was unable to move laterally to the top of the desired object')
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
		process_do_service(
			services.call_calculate_dimension(),
			'The arm was unable to calculate dimensions of object',
			'Object dimensions determined')
calculate_dimensions = CALCULATE_DIMENSIONS()

class PICK_OBJECT(abstract_state):
	def entry(self):
		print('Picking up object')
		process_do_service(
			services.call_pick_object(),
			'The arm was unable to approach the object')
	def exit(self):
		print('Checking if arm has picked the object')
		process_do_service(
			services.call_update_has_object(),
			'The arm was unable to determine if the object is held')
pick_object = PICK_OBJECT()

class RESTART_PICK_OBJECT(abstract_state):
	def entry(self):
		print('Arm was unable to pick up object, trying again...')
		process_do_service(
			services.call_restart_pick_object(),
			'The arm was unable to attempt another pick up')
restart_pick_object = RESTART_PICK_OBJECT()

class PLACE_OBJECT(abstract_state):
	def entry(self):
		print('Placing object at desired coord')
		process_do_service(
			services.call_place_object(),
			'The arm was unable to place the object')
	def exit(self):
		print('Setting task as complete')
		process_do_service(
			services.call_set_task_complete(),
			'The arm was unable to set the task as complete')
place_object = PLACE_OBJECT()

""""TRANSITION DEFINITIONS"""
class is_go(abstract_transition):
	def condition(self):
		return services.call_is_go()
class default(abstract_transition):
	def condition(self):
		return True
class has_object(abstract_transition):
	def condition(self):
		return services.call_has_object()
class not_has_object(abstract_transition):
	def condition(self):
		return services.call_not_has_object()
class is_task_empty(abstract_transition):
	def condition(self):
		return services.call_is_task_empty()
class not_is_task_empty(abstract_transition):
	def condition(self):
		return services.call_not_is_task_empty()
"""THE LOGIC OF THE STATEFLOW STARTS HERE
THE FINITE STATE MACHINE IS AN ADJACENCY LIST.
"""
finite_state_machine = {
	neutral_pose : [is_go(locate_all_object)],
	locate_all_object : [default(set_desired_object)],
	set_desired_object : [default(lateral_move)],
	lateral_move : [default(ensure_is_on_top)],
	ensure_is_on_top : [default(calculate_dimensions)],
	calculate_dimensions : [default(pick_object)],
	pick_object : [has_object(place_object), not_has_object(restart_pick_object)],
	restart_pick_object : [default(set_desired_object)],
	place_object : [is_task_empty(neutral_pose), not_is_task_empty(set_desired_object)]
}
