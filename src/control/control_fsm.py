from abc import ABC, abstractmethod 
import time

"""ABSTRACT STATES AND TRANSITIONS"""
class abstract_state(ABC):
	@abstractmethod
	def entry(self):
		pass
class abstract_transition(ABC):
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
	
class transition1(abstract_transition):
	def condition(self):
		return True
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
	neutral_pose : [transition1(locate_object)],
	locate_object : [transition2(set_desired_object)],
	set_desired_object : [transition3(neutral_pose)],
}
