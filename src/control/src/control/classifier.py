#!/usr/bin/env python

import rospy
from control.srv import isFieldAnalyzed, isFieldAnalyzedResponse
from control.msg import classifier_list

class classifier:
    def __init__(self):
        self.pub = rospy.Publisher('classifier_list', classifier_list, queue_size=10)
        self.serv = rospy.Service('is_field_analyzed', isFieldAnalyzed, self.analyze)
        self.objects = []
        self.x = 0
        self.y = 0
        self.obj_class = ""

    def analyze(self, req):
    	print('Searching field for what objects are present and where in the field')
        # set self.x, self.y, self.classes
        obj = object(5,5,"foo")
        self.objects.append(obj)
    	return True

    # add service here to update x, y, class

class object:
    def __init__(self, x, y, obj_class):
        self.x = x
        self.y = y
        self.obj_class = obj_class
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_class(self):
        return self.obj_class

if __name__ == '__main__':
    rospy.init_node('classifier')
    classifier = classifier()
    # main loop
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        classifier.pub.publish(classifier.x, classifier.y, classifier.obj_class)
        r.sleep();
