#!/usr/bin/env python

import rospy

from control.srv import isMoveComplete
from control.msg import object_lateral
from parameter_server import parameter_server

class MoveIt:
	def __init__(self):
		#should these 3 neutral params be in parameter server
		self.neutral_x, self.neutral_y, self.neutral_z = (0,0,0)#parameter_server.get_neutral_pose()
		self.object_lateral = rospy.Subscriber("object_lateral", object_lateral, self.set_desired_object_coord)
		self.lateral_move_serv = rospy.Service('lateral_move', isMoveComplete, self.lateral_move)
	
	def set_desired_object_coord(self, object_pose):
		self.desired_object_x = object_pose.x
		self.desired_object_y = object_pose.y 
	def get_desired_object_coord(self):
		return self.desired_object_x, self.desired_object_y
	
	def move_to_coord(self,x,y,z):
		pass
	def lateral_move(self, req):
		desired_x , desired_y = self.get_desired_object_coord()
		rospy.loginfo('desired object coords %f %f' % (desired_x, desired_y))
		#actually moving (unlike the rest of this function this will likely be changed)
		self.move_to_coord(desired_x, desired_y, self.neutral_z)
		
		return True

if __name__ == '__main__':
	rospy.init_node('move')
	move_it = MoveIt()
	rospy.spin()
	#r = rospy.Rate(10)
	#while not rospy.is_shutdown():
	#	r.sleep()

	