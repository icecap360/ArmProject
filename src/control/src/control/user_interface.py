#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import class_list
from control.srv import isGo, isGoResponse

pub = rospy.Publisher('desired_classes', class_list, queue_size=10)

# anonymous to append numbers to class_list to make it unique in case of collision
rospy.init_node('desired_classes', anonymous=True)
r = rospy.Rate(10)

# define service
def is_go(req):
  return isGoResponse(1)
s = rospy.Service('is_go', isGo, is_go)

# service call
# wait_for_service(<name of service>)
#rospy.wait_for_service('is_go')
# ServiceProxy(<function to be called>, <service name from above import>)
# makes service function into local function
#is_go = rospy.ServiceProxy('is_go', isGo)

while not rospy.is_shutdown():
    pub.publish(["foo", "bar"])
    #print(add_two_ints(1,2))
    r.sleep();
