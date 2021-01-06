#!/usr/bin/env python

import rospy
from control.srv import (
    doService,
    isTaskEmpty,
    segmentComplete
)
from control.msg import object_lateral

class armVision:
    def __init__(self):
        self.object_lateral_topic = rospy.Publisher('object_lateral', object_lateral, queue_size=10)
        self.analyze_serv = rospy.Service('locate_all_objects', doService, self.locate_all_objects)
        self.set_object_serv = rospy.Service('set_object', doService, self.set_object)
        self.complete_task_serv = rospy.Service('set_task_complete', doService, self.set_task_complete)
        self.is_empty_serv = rospy.Service('is_task_empty', isTaskEmpty, self.is_task_empty)
        self.is_segment_complete_serv = rospy.Service('set_segment_complete', segmentComplete, self.set_segment_complete)
        self.object_list = []
        self.object_list_empty = True
        self.x = 0
        self.y = 0
        self.obj_class = ""
        self.segment_complete = False

    def get_x(self):
        return self.x
    def get_y(self):
        return self.y
    def get_obj_class(self):
        return self.obj_class
    def get_object_list(self):
        return self.object_list
    def get_object_list_empty(self):
        return self.object_list_empty
    def get_segment_complete(self):
        return self.segment_complete
    def set_x(self, x):
        self.x = x
    def set_y(self, y):
        self.y = y
    def set_obj_class(self, obj_class):
        self.obj_class = obj_class
    def set_object_list(self, object_list):
        self.object_list = object_list
    def set_object_list_empty(self, empty):
        self.object_list_empty = empty
    def set_segment_complete(self, complete):
        self.segment_complete = complete

    """ services """
    # set object as head of list
    def set_object(self, req):
        obj = self.get_object_list()[0]
        self.set_x(obj.get_x() )
        self.set_y(obj.get_y() )
        self.set_obj_class(obj.get_obj_class() )
        return True

    def locate_all_objects(self, req):
    	print('Searching field for what objects are present and where in the field')
        obj_list = self.get_object_list()
        obj = object(5,5,"foo")
        obj_list.append(obj)
        obj_list.append(obj)
        self.set_object_list(obj_list)
        self.set_object_list_empty(False)
    	return True

    def set_task_complete(self, req):
        obj_list = self.get_object_list()
        if len(obj_list) > 1:
            obj_list.pop(0)
            self.set_object_list(obj_list)
            self.set_object_list_empty(False)
        else:
            self.set_object_list_empty(True)
        return True

    def is_task_empty(self, req):
        return self.get_object_list_empty()

    def set_segment_complete(self, req):
        self.set_segment_complete(True)


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
    rospy.init_node('arm_vision')
    arm_vision = armVision()
    # main loop
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        arm_vision.object_lateral_topic.publish(
            arm_vision.get_x(), arm_vision.get_y(), arm_vision.get_obj_class() )
        r.sleep();
