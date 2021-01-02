import rospy

class parameterServer:
	def initialize(self):
		rospy.set_param('neutral_pose', {'x':1,'y':2,'z':3})
		rospy.set_param('lateral_error_tolerance', 100)
		rospy.set_param('camera_offset', 3)
	def get_neutral_pose(self):
		return rospy.get_param('/neutral_pose')
	def get_lateral_error_tolerance(self):
		return rospy.get_param('/lateral_error_tolerance')

parameter_server = parameterServer()