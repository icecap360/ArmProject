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

""""DEFINITIONS OF THE TRANSITION CONDITIONS STARTS HERE"""

def alwaysTrue():
	return True

"""THE LOGIC OF THE STATEFLOW STARTS HERE
THE FINITE STATE MACHINE IS AN ADJACENCY LIST.
IT IS IMPLEMENTED AS A DICTIONARY FROM STATE TO LIST OF TUPLES (TRANSITION_CONDITION, TRANSITION_END_STATE)
"""
finite_state_machine = {
	state1 : [(alwaysTrue,state2)],
	state2 : [(alwaysTrue,state3)],
	state3 : [(alwaysTrue,state1)],
}

""" MAIN MODULE STARTS HERE """
initState = state1
currentState = initState
while(1):
	transitions = finite_state_machine[currentState]
	time.sleep(3)
	for t in transitions:
		transition_condition = t[0]
		next_state = t[1]
		if transition_condition():
			if hasattr(currentState, 'exit'):
				currentState.exit()
			next_state.entry()
			currentState = next_state
			break
	else:
		if hasattr(currentState, 'during'):
			currentState.during()