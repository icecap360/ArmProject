#!/usr/bin/env python

import rospy
from control.srv import isFieldAnalyzed, isFieldAnalyzedResponse, setTaskComplete, setTaskCompleteResponse
from control.msg import classifier_list

class classifier:
    def __init__(self):
        self.pub = rospy.Publisher('classifier_list', classifier_list, queue_size=10)
        self.analyze_serv = rospy.Service('is_field_analyzed', isFieldAnalyzed, self.analyze)
        self.complete_task_serv = rospy.Service('set_task_complete', setTaskComplete, self.set_task_complete)
        self.objects = []
        self.x = 0
        self.y = 0
        self.obj_class = ""

    # set object as head of list
    def set_object(self):
        self.x = self.objects[0].get_x()
        self.y = self.objects[0].get_y()
        self.obj_class = self.objects[0].get_obj_class()

    # services
    def analyze(self, req):
    	print('Searching field for what objects are present and where in the field')
        obj = object(5,5,"foo")
        self.objects.append(obj)
        self.set_object()
    	return True

    def set_task_complete(self, req):
        self.objects = self.objects.pop(0)
        self.set_object()


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
