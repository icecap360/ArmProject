from abc import ABC, abstractmethod 
import time

class abstract_state(ABC):
	@abstractmethod
	def entry(self):
		pass

""""DEFINITIONS OF THE STATES STARTS HERE"""
class STATE1(abstract_state):
	def entry(self):
		print('Entering STATE1')
	def during(self):
		print('In STATE1')
	def exit(self):
		print('Exiting STATE1')
state1 = STATE1()

class STATE2(abstract_state):
	def entry(self):
		print('Entering STATE2')
state2 = STATE2()

class STATE3(abstract_state):
	def entry(self):
		print('Entering STATE3')
	def during(self):
		print('In STATE3')
state3 = STATE3()

""""DEFINITIONS OF THE TRANSITIONS, they feature a condition and an end state"""

class abstract_transition(ABC):
	def __init__(self, next_state):
		self.next_state = next_state
	@abstractmethod
	def condition(self):
		pass
	
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
IT IS IMPLEMENTED AS A DICTIONARY FROM STATE TO LIST OF TUPLES (TRANSITION_CONDITION, TRANSITION_END_STATE)
"""
finite_state_machine = {
	state1 : [transition1(state2)],
	state2 : [transition2(state3)],
	state3 : [transition3(state1)],
}

""" MAIN MODULE STARTS HERE """
initState = state1
currentState = initState
while(1):
	transitions = finite_state_machine[currentState]
	time.sleep(3)
	for t in transitions:
		if t.condition():
			if hasattr(currentState, 'exit'):
				currentState.exit()
			t.next_state.entry()
			currentState = t.next_state
			break
	else:
		if hasattr(currentState, 'during'):
			currentState.during()