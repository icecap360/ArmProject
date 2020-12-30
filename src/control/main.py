from control_fsm import state1, finite_state_machine
import time
#print(dir(control_fsm))
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