import rospy

def get_neutral_pose():
	return rospy.get_param('/neutral')
def get_lateral_error_tolerance():
	return rospy.get_param('/error_tolerance/lateral')
