#! /usr/bin/env python

from control_fsm import neutral_pose, finite_state_machine, services
import rospy 
import time
rospy.init_node('Main')
services.initialize()
initState = neutral_pose
currentState = initState
currentState.entry()
while(1):
	if rospy.is_shutdown():
		break
	transitions = finite_state_machine[currentState]
	time.sleep(1)
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