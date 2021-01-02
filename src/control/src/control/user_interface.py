#! /usr/bin/env python

import rospy
from control.msg import class_list, arm_parameters
from control.srv import isGo, isGoResponse

class userInterface:
    def __init__(self):
        self.desired_classes = rospy.Publisher('desired_classes', class_list, queue_size=10)
        self.serv = rospy.Service('is_go', isGo, self.is_go)
        self.desired_class_list = [""]

    def get_desired_classes(self):
        self.set_desired_classes(raw_input("Enter space seperated desired classes: ").split() )
        return self.start()
    def set_desired_classes(self, desired_classes):
        self.desired_classes_list = desired_classes
    def start(self):
        if raw_input("Start robot? (y/n) ") == 'y':
            return True
        return False

    """ service """
    def is_go(self, req):
        return self.get_desired_classes()

if __name__ == '__main__':
    rospy.init_node('user_interface', anonymous=True)
    # call constructor
    user_interface = userInterface()
    # main loop
    r = rospy.Rate(10) #this should be decreased, arm_params is quite big
    while not rospy.is_shutdown():
        user_interface.desired_classes.publish(user_interface.desired_class_list)
        r.sleep();
