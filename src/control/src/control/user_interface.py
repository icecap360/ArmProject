#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import class_list
from control.srv import isGo, isGoResponse
#import string

class userInterface:
    def __init__(self):
        self.pub = rospy.Publisher('desired_classes', class_list, queue_size=10)
        self.serv = rospy.Service('is_go', isGo, self.is_go)
        self.desired_classes = [""]

    # define service
    def is_go(self, req):
        return self.get_desired_classes()

    def get_desired_classes(self):
        self.desired_classes = raw_input("Enter space seperated desired classes: ").split()
        return self.start()

    def start(self):
        if raw_input("Start robot? (y/n) ") == 'y':
            return True
        return False


if __name__ == '__main__':
    rospy.init_node('user_interface', anonymous=True)
    # call constructor
    user_interface = userInterface()
    # main loop
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        user_interface.pub.publish(user_interface.desired_classes)
        #print(add_two_ints(1,2))
        r.sleep();
