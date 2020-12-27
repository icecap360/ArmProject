from std_msgs import String
import rospy

pub = rospy.Publisher('foo', String, queue_size=10)

rospy.init_node('foo node')
r = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish("foo")
    r.sleep();
