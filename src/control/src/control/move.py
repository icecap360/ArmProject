#!/usr/bin/env python

import rospy

from params import get_neutral_pose
from control.msg import object_lateral, arm_parameters
from control.srv import isMoveComplete, moveToNeutralPose

class MoveIt:
	def __init__(self):
		#should these 3 neutral params be in parameter server
		self.object_lateral = rospy.Subscriber("object_lateral", object_lateral, self.set_desired_object_coord)
		self.lateral_move_serv = rospy.Service('lateral_move', isMoveComplete, self.lateral_move)
		self.pick_object_serv = rospy.Service('pick_object', isMoveComplete, self.pick_object )
		self.move_to_neutral_serv = rospy.Service('move_to_neutral_pose', moveToNeutralPose, self.move_to_neutral_pose)
		self.neutral_x ,  self.neutral_y , self.neutral_z = get_neutral_pose()

	def set_desired_object_coord(self, object_pose):
		self.desired_object_x = object_pose.x
		self.desired_object_y = object_pose.y
	def get_desired_object_coord(self):
		return self.desired_object_x, self.desired_object_y
	def get_neutral(self):
		return self.neutral_x, self.neutral_y, self.neutral_z

	def move_to_coord(self,x,y,z):
		pass

	def move_to_neutral_pose(self, req):
		x, y, z = self.get_neutral()
		self.move_to_coord(x, y, z)
		return True

	def lateral_move(self, req):
		desired_x , desired_y = self.get_desired_object_coord()

		#actually moving (unlike the rest of this function this will likely be changed)
		self.move_to_coord(desired_x, desired_y, self.neutral_z)

		return True
	def pick_object(self,req):
		#approach down diagonal
		#approach lateral
		#grab object
		#pick_up object
		return True

if __name__ == '__main__':
	rospy.init_node('move')
	move_it = MoveIt()
	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		r.sleep()
