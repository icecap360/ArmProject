#! /usr/bin/env python

from control_fsm import (
	neutral_pose, finite_state_machine,
	services, has_exit_commands,
	has_during_commands
)
import rospy

if __name__ == '__main__':
	rospy.init_node('Main')
	services.initialize()
	initState = neutral_pose
	currentState = initState
	currentState.entry()
	r = rospy.Rate(2)
	while not rospy.is_shutdown():
		transitions = finite_state_machine[currentState]
		r.sleep()
		for t in transitions:
			if t.condition():
				if has_exit_commands(currentState):
					currentState.exit()
				t.next_state.entry()
				currentState = t.next_state
				break
		else:
			if has_during_commands(currentState):
				currentState.during()
