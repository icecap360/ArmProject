#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import class_list
from control.srv import isGo, isGoResponse

class user_interface:
    def __init__(self):
        self.pub = rospy.Publisher('desired_classes', class_list, queue_size=10)
        self.serv = rospy.Service('is_go', isGo, self.is_go)

    # define service
    def is_go(self, req):
      return isGoResponse(1)

if __name__ == '__main__':
    rospy.init_node('desired_classes', anonymous=True)
    # call constructor
    ui = user_interface()
    # main loop
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        ui.pub.publish(["foo", "bar"])
        #print(add_two_ints(1,2))
        r.sleep();
