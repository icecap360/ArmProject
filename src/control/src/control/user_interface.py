#! /usr/bin/env python

import rospy
# from std_msgs.msg import String
from control.msg import class_list

pub = rospy.Publisher('desired_classes', class_list, queue_size=10)

# anonymous to append numbers to class_list to make it unique in case of collision
rospy.init_node('desired_classes', anonymous=True)
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(["foo", "bar"])
    r.sleep();
