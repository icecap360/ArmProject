#! /usr/bin/env python

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from control.msg import image_points
from control.srv import doService
import cv2
import random
import pyclipper
# import os
# import time
import rospkg

class imageSegmenter:
    def __init__(self):
        rospack = rospkg.RosPack()
        dir = rospack.get_path('control') + "/src/control/"
        self.point_cloud_shape = (480,640)
        self.yolo_shape = (416,416)
        self.weight_path= dir + "models/yolov3.weights"
        self.config_path= dir + "models/yolov3.cfg"
        self.labels_path= dir + "models/coco.names"
        self.confidence = 0.5
        self.threshold = 0.3
        self.NMS = True
        self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weight_path)
        self.labels = open(self.labels_path).read().strip().split('\n')
        self.execute = True #set this to false so node does not initially execute
        # All SERVICES and TOPICS MUST be created BELOW
        self.sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.classify_img)
        self.serv = rospy.Service('get_image_hulls', doService, self.get_image_hulls_srvcb)
        self.pub = rospy.Publisher('image_hulls', image_points, queue_size=1)
        self.image_pub = rospy.Publisher('arm_vision_image', Image, queue_size=1)

    def get_image_hulls_srvcb(self, req):
        self.execute=True
        return True

    def classify_img(self, ros_cloud):
        if not self.execute:
            return
        ros_cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(ros_cloud)
        xyz = ros_numpy.point_cloud2.get_xyz_points(ros_cloud_arr, remove_nans=False)
        rgb = ros_numpy.point_cloud2.split_rgb_field(ros_cloud_arr)
        img = np.array([rgb['r'],rgb['g'],rgb['b']]) #shape is 3,480,640
        img = np.moveaxis(img, 0, 2) #shape is 480,640,3
        self.height, self.width = img.shape[:2]
        boxes, confidences, classes = self.make_prediction(self.net, self.labels,img,self.confidence,self.threshold)
        image_segments = self.make_contour_hulls(img, boxes)
        # --- turn each contour hull into a xyz coordinate ---
        # publish that list of convex hulls (same size/indexing as boxes)
        #self.pub_predictions(self.boxes,self.classes)
    
        self.execute = True #this must be false
        print("Finished execution callback")

    def extract_boxes_confidences_classids(self,outputs, confidence, width, height):
        # Helper for make_predictions
        boxes = []
        confidences = []
        classIDs = []
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                conf = scores[classID]
                if conf > confidence:
                    box = detection[0:4] * np.array([width, height, width, height])
                    centerX, centerY, w, h = box.astype('int')
                    boxes.append([centerX, centerY, w, h])
                    confidences.append(float(conf))
                    classIDs.append(classID)
        return boxes, confidences, classIDs

    def make_prediction(self, net, labels, image, confidence, threshold):
        layer_names = net.getLayerNames()
        layer_names = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(layer_names)
        boxes, confidences, classIDs = self.extract_boxes_confidences_classids(outputs, confidence, width, height)
        # If requested, further combine similar bounding boxes using NMS
        if self.NMS:
            idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)
            if len(idxs)>0:
                idxs = idxs.flatten()
                boxes = self.subset_detections(boxes, idxs)
                confidences = self.subset_detections(confidences, idxs)
                classIDs = self.subset_detections(classIDs, idxs)
        classes = [labels[cid] for cid in classIDs]
        return boxes, confidences, classes

    def make_contour_hulls(self, img, boxes):
        hulls = []
        for box in boxes:
            centerX,centerY,w,h = box[0],box[1],box[2],box[3]
            top_left_row,top_left_col = self.pixel_to_index((centerX - (w//2) , centerY - (h//2)))
            bot_right_row, bot_right_col = self.pixel_to_index((centerX + (w//2), centerY + (h//2)))
            image_cropped =  img[
                top_left_row:bot_right_row,
                top_left_col:bot_right_col, :]
            old_image_cropped = image_cropped.copy()
            
            #Binarize the image, so that the background is ignored. There are 3 ways
            #1. Specifically threshold the image to the gray background
                #so for example is an image showed veriation in any channel away from gray, it would show.
            #2. options include filter small contours (smaller contours typically in background)
            #3. (We choose this) threshold the image after turning it gray, this option is common online, perhaps less influenced by background color 
            gray=cv2.cvtColor(image_cropped,cv2.COLOR_BGR2GRAY)
            thresh = cv2.threshold(gray, 100 ,255, cv2.THRESH_BINARY)[1] #125 is threshold online, it did not work, so I choose 100
            
            #in thresholding anything not gray is black (so the object is typically black)
            #dilation shrinks the black portion of an image
            kernel = np.ones((10,10),np.uint8)
            shrinked = cv2.dilate(thresh,kernel,iterations = 1)
            
            edged=cv2.Canny(shrinked,30,200)
            #Now find all the contours
            contours, hierarchy=cv2.findContours(edged,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
            #There is only one object, so all the contours have to do with one object            
            all_contours = np.concatenate(contours)
            #Using convex hulls, alot of the interior shape is lost, so instaed a polydon was formed
            epsilon = 0.0004*cv2.arcLength(all_contours,True) # 0.001 gives 90 points, 0.005 gives 70
            hull = cv2.approxPolyDP(all_contours,epsilon,True)
            
            #offset hull back into the orignal image
            hull[:,:,0] += top_left_col
            hull[:,:,1] += top_left_row
            
            hulls.append(hull)
            
        #draw the contours and publish the image
        hullsss = np.concatenate(np.array(hulls))
        print hullsss.shape
        image_cropped = cv2.UMat(img)
        cv2.drawContours(image_cropped,hullsss,-1,(0,255,0),15)
        image_cropped = image_cropped.get()
        self.image_pub.publish(
            ros_numpy.image.numpy_to_image(
            image_cropped, "rgb8"))

        return hulls

    def subset_detections(self, l , indexes):
        return [l[i] for i in indexes]
    def pixel_to_index(self, coord):
        x = int(coord[0])
        col_ind = min(x,self.width-1)
        col_ind = max(col_ind,0)
        y = int(coord[1])
        row_ind = min(y,self.height-1)
        row_ind = max(row_ind,0)
        return row_ind, col_ind
    def pixle_to_xy(self, coord):
        row_ind, col_ind = self.pixel_to_index(coord)
        #why does this execute 16 times?
        return self.xyz[row_ind][col_ind][0],self.xyz[row_ind][col_ind][1]
    def get_coordinates(self,box):
        #converts box pixle coordinates to the x,y  coordinates
        centerX,centerY,w,h = box[0],box[1],box[2],box[3]
        top_left = self.pixle_to_xy((centerX - (w/2) , centerY - (h/2)))
        top_right = self.pixle_to_xy((centerX + (w/2), centerY - (h/2)))
        bot_left = self.pixle_to_xy((centerX - (w/2) , centerY + (h/2)))
        bot_right = self.pixle_to_xy((centerX + (w/2), centerY + (h/2)))
        return (top_left,top_right,bot_left,bot_right)
    def pub_predictions(self, det_boxes, det_classes):
        # Turns the raw bounding boxes and classes into a msg, and publishes it
        msg = image_points()
        msg.x, msg.y, msg.obj_class = [],[],[]
        end_of_det_num = float('inf') #this should come from the param server
        end_of_det_str = ''
        for i in range(len(det_boxes)):
            coords = self.get_coordinates(det_boxes[i])
            for coord in coords:
                msg.x.append(coord[0])
                msg.y.append(coord[1])
            msg.obj_class.append(det_classes[i])
            #mark the end of detection
            msg.x.append(end_of_det_num)
            msg.y.append(end_of_det_num)
            msg.obj_class.append(end_of_det_str)
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('image_segmenter', anonymous=True)
    image_segmenter = imageSegmenter()
    # main loop
    rospy.spin()
