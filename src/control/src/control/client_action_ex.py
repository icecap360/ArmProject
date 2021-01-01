#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from control.msg import ensureIsOnTopGoal, ensureIsOnTopAction

def is_on_top_client():
	# Creates the SimpleActionClient, passing the type of the action
	# (FibonacciAction) to the constructor.
	client = actionlib.SimpleActionClient('ensure_is_on_top', ensureIsOnTopAction)

	# Waits until the action server has started up and started
	# listening for goals.
	client.wait_for_server()

	# Creates a goal to send to the action server.
	goal = ensureIsOnTopGoal()

	# Sends the goal to the action server.
	client.send_goal(goal)

	# Waits for the server to finish performing the action.
	client.wait_for_result()

	# Prints out the result of executing the action
	return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
	rospy.init_node('is_on_top_client_client_py')
	result = is_on_top_client()
	print(result, result.is_on_top)
