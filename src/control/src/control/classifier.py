#!/usr/bin/env python

import rospy
from control.srv import isFieldAnalyzed, \
    setTaskComplete, setTaskCompleteResponse, \
    setObject
from control.msg import classifier_list

class classifier:
    def __init__(self):
        self.pub = rospy.Publisher('classifier_list', classifier_list, queue_size=10)
        self.analyze_serv = rospy.Service('is_field_analyzed', isFieldAnalyzed, self.analyze)
        self.set_object_serv = rospy.Service('set_object', setObject, self.set_object)
        self.complete_task_serv = rospy.Service('set_task_complete', setTaskComplete, self.set_task_complete)
        self.object_list = []
        self.x = 0
        self.y = 0
        self.obj_class = ""

    # services

    # set object as head of list
    def set_object(self, req):
        self.x = self.object_list[0].get_x()
        self.y = self.object_list[0].get_y()
        self.obj_class = self.object_list[0].get_obj_class()
        return True

    def analyze(self, req):
    	print('Searching field for what objects are present and where in the field')
        obj = object(5,5,"foo")
        self.object_list.append(obj)
        #self.object_list.append(obj)
        #self.set_object()
    	return True

    # complete
    def set_task_complete(self, req):
        response = setTaskCompleteResponse()
        if len(self.object_list) > 1:
            self.object_list.pop(0)
            response.is_empty = False
            #self.set_object()
        response.is_empty = True
        response.success = True
        return response


class object:
    def __init__(self, x, y, obj_class):
        self.x = x
        self.y = y
        self.obj_class = obj_class
    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_obj_class(self):
        return self.obj_class

if __name__ == '__main__':
    rospy.init_node('classifier')
    classifier = classifier()
    # main loop
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        classifier.pub.publish(classifier.x, classifier.y, classifier.obj_class)
        r.sleep();
