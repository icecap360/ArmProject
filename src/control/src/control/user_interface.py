#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import class_list, arm_parameters
from control.srv import isGo, isGoResponse
#import string

class userInterface:
    def __init__(self):
        self.desired_classes = rospy.Publisher('desired_classes', class_list, queue_size=10)
        self.arm_parameter_server = rospy.Publisher('arm_parameter_server', arm_parameters, queue_size=10)
        self.serv = rospy.Service('is_go', isGo, self.is_go)
        self.desired_class_list = [""]
        self.arm_parameters = self.default_arm_parameters()

    # define service
    def is_go(self, req):
        return self.get_desired_classes()

    def get_desired_classes(self):
        self.desired_class_list = raw_input("Enter space seperated desired classes: ").split()
        return self.start()

    def start(self):
        if raw_input("Start robot? (y/n) ") == 'y':
            return True
        return False
    def default_arm_parameters(self):
        arm_params = arm_parameters()
        arm_params.neutral_x = 1
        arm_params.neutral_y = 2
        arm_params.neutral_z = 3
        arm_params.lateral_error_tolerance = 4
        arm_params.camera_offset = 5
        return arm_params



if __name__ == '__main__':
    rospy.init_node('user_interface', anonymous=True)
    # call constructor
    user_interface = userInterface()
    # main loop
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        user_interface.desired_classes.publish(user_interface.desired_class_list)
        user_interface.arm_parameter_server.publish(user_interface.arm_parameters)
        #print(add_two_ints(1,2))
        r.sleep();
