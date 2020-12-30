from control_fsm import neutral_pose, finite_state_machine
import time

initState = neutral_pose
currentState = initState
currentState.entry()
while(1):
	transitions = finite_state_machine[currentState]
	time.sleep(2.5)
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