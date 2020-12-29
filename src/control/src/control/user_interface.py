#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import class_list
from control.srv import AddTwoInts

pub = rospy.Publisher('desired_classes', class_list, queue_size=10)

# anonymous to append numbers to class_list to make it unique in case of collision
rospy.init_node('desired_classes', anonymous=True)
r = rospy.Rate(10)

# service call
# wait_for_service(<name of service>)
rospy.wait_for_service('add_two_ints')
# ServiceProxy(<function to be called>, <service name from above import>)
# makes service function into local function
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)

while not rospy.is_shutdown():
    pub.publish(["foo", "bar"])
    print(add_two_ints(1,2))
    r.sleep();
