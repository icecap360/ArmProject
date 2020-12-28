#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import desired_classes

pub = rospy.Publisher('class_list', desired_classes, queue_size=10)

# anonymous to append numbers to class_list to make it unique in case of collision
rospy.init_node('class_list', anonymous=True)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(["foo", "bar"])
    r.sleep();
