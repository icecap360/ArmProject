#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import class_list
from control.srv import isGo

pub = rospy.Publisher('desired_classes', class_list, queue_size=10)

# anonymous to append numbers to class_list to make it unique in case of collision
rospy.init_node('client', anonymous=True)
r = rospy.Rate(10)

# service call
# wait_for_service(<name of service>)
rospy.wait_for_service('is_go')
# ServiceProxy(<function to be called>, <service name from above import>)
# makes service function into local function
is_go = rospy.ServiceProxy('is_go', isGo)

while not rospy.is_shutdown():
    pub.publish(["foo", "bar"])
    print(is_go())
    r.sleep();
