#! /usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('class_list', String, queue_size=10)

# anonymous to append numbers to class_list to make it unique in case of collision
rospy.init_node('class_list', anonymous=True)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish("foo")
    r.sleep();
