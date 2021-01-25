#!/usr/bin/env python

import rospy
from control.srv import (
    doService,
    isTaskEmpty,
    segmentComplete
)
from control.msg import (
    object_lateral,
    cluster_points,
    image_points
)
import numpy as np
import cv2
import munkres

class armVision:
    def __init__(self):
        rospy.wait_for_service('get_pcl_hulls')
        self.get_pcl_hulls = rospy.ServiceProxy('get_pcl_hulls', doService)
        rospy.wait_for_service('get_image_hulls')
        self.get_image_hulls = rospy.ServiceProxy('get_image_hulls', doService)

        self.object_lateral_pub = rospy.Publisher('object_lateral', object_lateral, queue_size=10)
        self.cloud_hull_sub = rospy.Subscriber('cloud_hulls', cluster_points, self.pcl_segment_complete_subcb)
        self.image_hull_sub = rospy.Subscriber('image_hulls', image_points, self.image_segment_complete_subcb)
        self.analyze_serv = rospy.Service('locate_all_objects', doService, self.locate_all_objects)
        self.set_object_serv = rospy.Service('set_object', doService, self.set_object)
        self.complete_task_serv = rospy.Service('set_task_complete', doService, self.set_task_complete)
        self.is_empty_serv = rospy.Service('is_task_empty', isTaskEmpty, self.is_task_empty)
        # self.pcl_segment_complete_serv = rospy.Service('pcl_segment_complete', segmentComplete, self.pcl_segment_complete)


        self.object_list = []
        self.object_list_empty = True
        self.x = 0
        self.y = 0
        self.obj_class = ""
        self.pcl_segment_complete = False
        self.cluster_points = cluster_points()
        self.image_segment_complete = False
        self.image_hulls = image_points()
        self.cluster_center = [0]

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
    def get_pcl_segment_complete(self):
        return self.pcl_segment_complete
    def get_pcl(self):
        return self.pcl_hulls
    def get_image_segment_complete(self):
        return self.image_segment_complete
    def get_image(self):
        return self.image_hulls
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
    def set_pcl_segment_complete(self, complete):
        self.pcl_segment_complete = complete
    def set_pcl(self, pcl):
        self.pcl_hulls = pcl
    def set_image_segment_complete(self, complete):
        self.image_segment_complete = complete
    def set_image(self, image):
        self.image_hulls = image

    # shoelace algorithm
    # requires x,y to be sorted CW or CCW
    def calc_area(self, x, y):
        if len(x) == 0 or x[0] == None:
            return 0
        # print(x)
        # print(y)
        return 0.5*np.abs(np.dot(x,np.roll(y,1))-np.dot(y,np.roll(x,1)))

    # convex hull
    def get_hull(self, points):
        if len(points) == 0:
            return np.array( [[None]*2] )
        hull = cv2.convexHull(points)
        hull = np.squeeze(hull)
        return hull

    # converts list of x,y into (x,y) split by inf
    def convert_to_coordinates(self, x, y):
        x = np.array(x)
        y = np.array(y)
        coord = np.column_stack( (x,y) )
        indices = np.where(x!=float('inf') ) [0]
        coord = np.array_split(coord[indices][:], np.where(coord==float('inf') ) [0][:])
        # returns array of arrays of different length
        # each subarray represents one cluster
        #print(coord[0])
        # get number of clusters
        clusters = np.size(x) - np.size(indices)
        return coord, clusters

    # comparator callback for clockwise sorting
    # return -1 -> a before b
    #         1 -> a after b
    #         0 -> equal preference
    def cw_cmp(self, a, b):
        center = self.center
        # simple cases
        if (a[0] - center[0]) >= 0 and (b[0] - center[0]) < 0:
            return 1
        if (a[0] - center[0]) < 0 and (b[0] - center[0]) >= 0:
            return -1
        if (a[0] - center[0]) == 0 and (b[0] - center[0]) == 0:
            if (a[1] - center[1]) >= 0 and (b[1] - center[1]) >= 0:
                return a[1] > b[1]
            return b[1] > a[1]
        # cross product to find theta angle in polar coordinates
        det = ( (a[0] - center[0]) * (b[1] - center[1]) ) - ( (b[0] - center[0]) * (a[1] - center[1]) )
        if det < 0:
            return 1
        if det > 0:
            return -1
        # points lie on same line, therefore sort by distance
        d1 = ( (a[0] - center[0]) * (a[0] - center[0]) ) - ( (a[1] - center[1]) * (a[1] - center[1]) )
        d2 = ( (b[0] - center[0]) * (b[0] - center[0]) ) - ( (b[1] - center[1]) * (b[1] - center[1]) )
        return int(d1 > d2)

    # returns list sorted in clockwise order
    def sort_cw(self, points):
        self.center = points.mean(0)
        points = sorted(points, cmp=self.cw_cmp)
        return np.float32(points)

    # processes pcl and image clusters and combines them
    def create_objects(self, pcl_hulls, image_hulls):
        # algorithm:
        # get points of coordinates
        # restructure into x by y by cluster array
        # calc_area of each cluster
        # for each pair between pcl cluster and image cluster:
        #   combine into one cluster
        #   get hull
        #   find area of new cluster (union)
        #   find intersection / union
        # munkres algorithm to find min cost solution
        #   pcl_0 [image_0 image_1 ...]
        #   pcl_1 [image_0 image_1 ...]

        # convert raw messages into coordinates of (x,y) array
        pcl_coord, pcl_num_clusters = self.convert_to_coordinates(pcl_hulls.hull_x, pcl_hulls.hull_y)
        image_coord, image_num_clusters = self.convert_to_coordinates(image_hulls.x, image_hulls.y)

        # construct matrix for munkres algorithm
        # columns -> image_clusters
        # rows -> pcl_clusters
        # cost -> intersection over union
        cost = [[None]*image_num_clusters for i in range(pcl_num_clusters) ]
        # for each pcl cluster
        for i in range(0, pcl_num_clusters):
            # convert to cv2 point type
            # note: empty array between each cluster
            pcl_points = np.float32(pcl_coord[i*2])
            # calculate area of pcl cluster
            pcl_area = self.calc_area(pcl_points[:,0], pcl_points[:,1])
            # print("pcl area: ", pcl_area, " num points: ", len(pcl_points) )
            for j in range(0, image_num_clusters):
                image_points = np.float32(image_coord[j*2])
                image_area = self.calc_area(image_points[:,0], image_points[:,1])

                # calculate area of union
                union_points = np.vstack((pcl_points, image_points))
                # testing code
                # union_hull = self.get_hull(union_points)
                # union_area = self.calc_area(union_hull[:,0], union_hull[:,1])
                # print("union area before sort: ", union_area)
                # must sort union_points in CW or CCW order before finding hull
                union_points = self.sort_cw(union_points)
                union_hull = self.get_hull(union_points)
                union_area = self.calc_area(union_hull[:,0], union_hull[:,1])
                # calculate intersection over union
                if union_area == 0:
                    iou = 0
                else:
                    # calculate intersection over union
                    # note: P(AuB) = P(A) + P(B) - P(AnB)
                    intersection_area = pcl_area + image_area - union_area
                    iou = intersection_area/union_area
                # print("image area: ", image_area, " num points: ", len(image_points) )
                # print("union area: ", union_area, " num points: ", len(union_points))
                # print("intersection area: ", intersection_area)
                # print("iou: ", iou)
                # add iou into cost matrix
                cost[i][j] = iou
        # find lowest cost solution
        munk = munkres.Munkres()
        # indices is list of tuples of (i,j)
        indices = munk.compute(cost)
        #print("cost matrix: ", cost, "indices: ", indices)

        # create final objects
        obj_list = self.get_object_list()
        for i in range(len(indices)):
            obj = object(
                pcl_hulls.centroid_x[i*2], pcl_hulls.centroid_y[i*2], image_hulls.obj_class[i*2])
            obj_list.append(obj)
            print("object added: ", obj.get_x(), obj.get_y(), obj.get_obj_class() )
        self.set_object_list(obj_list)
        self.set_object_list_empty(False)



    """ subscriber callbacks """
    def pcl_segment_complete_subcb(self, cloud_hull):
        self.set_pcl_segment_complete(True)
        self.set_pcl(cloud_hull)
        print("Pcl hulls obtained.")
    def image_segment_complete_subcb(self, image_hull):
        self.set_image_segment_complete(True)
        self.set_image(image_hull)
        print("Image hulls obtained.")

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
        # obj_list = self.get_object_list()
        # obj = object(5,5,"foo")
        # obj_list.append(obj)
        # obj_list.append(obj)
        # self.set_object_list(obj_list)
        # self.set_object_list_empty(False)

        # start timer for segmentation limit time
        time = rospy.get_time()
        # start execution of segmenters
        success = self.get_pcl_hulls().success
        if not success:
    		print("Could not get pcl hulls.")
        else:
            print("Execute pcl segmentation called.")
        success = self.get_image_hulls().success
        if not success:
    		print("Could not get image hulls.")
        else:
            print("Execute image segmentation called.")

        r = rospy.Rate(10)
        while not (self.get_pcl_segment_complete() and
                   self.get_image_segment_complete() ):
            if rospy.get_time()-time > 5:
                print("Time limit exceeded.")
                # return false response in service?
                break
            r.sleep()
        # reset all flags of complete
        self.set_pcl_segment_complete(False)
        self.set_image_segment_complete(False)

        # compare bounding boxes and create object_list
        self.create_objects(self.get_pcl(), self.get_image() )

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
        arm_vision.object_lateral_pub.publish(
            arm_vision.get_x(), arm_vision.get_y(), arm_vision.get_obj_class() )
        r.sleep();
