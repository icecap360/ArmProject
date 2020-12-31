#!/usr/bin/env python

import rospy

from control.srv import isMoveComplete

class MoveIt:
	def __init__(self):
		#should these 3 neutral params be in parameter server
		self.neutral_x = 0
		self.neutral_y = 0
		self.neutral_z = 0
		self.lateral_move_serv = rospy.Service('lateral_move', isMoveComplete, self.lateral_move)
		
	def move_to_coord(self,x,y,z):
		pass
	def lateral_move(self, req):
		#read from topic the xy coord
		desired_x = 0
		desired_y = 0
		
		#actually moving (unlike the rest of this function this will likely be changed)
		self.move_to_coord(desired_x, desired_y, self.neutral_z)
		
		return True

if __name__ == '__main__':
	rospy.init_node('move')
	move_it = MoveIt()
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()

	