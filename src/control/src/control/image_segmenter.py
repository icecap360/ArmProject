#! /usr/bin/env python

import rospy
from control.msg import class_list, arm_parameters
from control.srv import isGo
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import cv2
import os
import time

class imageSegmenter:
    def __init__(self):

        self.weight_path= "models/yolov3.weights"
        self.config_path= "models/yolov3.cfg"
        self.labels_path= "models/coco.names"
        self.confidence = 0.5
        self.threshold = 0.3
        self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weight_path)
        layer_names = self.net.getLayerNames()
        self.layer_names = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        self.labels = open(self.labels_path).read().strip().split('\n')
        # All SERVICES and TOPICS MUST be created BELOW
        self.sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.saveImg)
        self.execute =True       
        print('yah')
        self.image_pub = rospy.Publisher('arm_vision_image', Image, queue_size=1)

    def saveImg(self, ros_cloud):
        if not self.execute:
            return
        ros_cloud_arr = ros_numpy.point_cloud2.pointcloud2_to_array(ros_cloud)
        xyz = ros_numpy.point_cloud2.get_xyz_points(ros_cloud_arr, remove_nans=False)
        rgb = ros_numpy.point_cloud2.split_rgb_field(ros_cloud_arr)
        #print(xyz.shape)
        r = rgb['r']
        g = rgb['g']
        b=rgb['b']
        img = np.array([r,g,b]) #shape is 3,480,640
        img = np.moveaxis(img, 0,2) #shape is 480,640,3
        self.img = img
        #print('The current directory of image segmenter is: ')
        #print( os.getcwd())
        self.yolo()
        self.execute = False #this must be false

    def yolo(self):
        self.img = cv2.resize(self.img,(416,416))#reshape to 416*416 as per blob
        self.boxes, self.confidences, self.classIDs, self.idxs = self.make_prediction(
            self.net, self.layer_names, self.labels, self.img, self.confidence, self.threshold)
        self.classes = [self.labels[cid] for cid in self.classIDs]
        #idxs contains tha real indexes we plan to keep, this subset should be done in yolo
        print(self.boxes)
        print(self.idxs)
        ### The code below is image printing code, it should be removed in the future
        colors = np.random.randint(0, 255, size=(len(self.labels), 3), dtype='uint8')
        imageBounded = self.draw_bounding_boxes(self.img, self.boxes, self.confidences, self.classIDs, self.idxs, colors, self.labels)
        cv2.imshow('YOLO Object Detection', self.img)
        self.image_pub.publish(
            ros_numpy.image.numpy_to_image(
                imageBounded, "rgb8"))
    #
    # def rotate_image(self, image, angle):
    #     image_center = tuple(np.array(image.shape[1::-1]) / 2)
    #     rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    #     result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    #     return result

    def extract_boxes_confidences_classids(self,outputs, confidence, width, height):
        boxes = []
        confidences = []
        classIDs = []
        for output in outputs:
            for detection in output:
                # Extract the scores, classid, and the confidence of the prediction
                scores = detection[5:]
                classID = np.argmax(scores)
                conf = scores[classID]
                # Consider only the predictions that are above the confidence threshold
                if conf > confidence:
                    # Scale the bounding box back to the size of the image
                    box = detection[0:4] * np.array([width, height, width, height])
                    centerX, centerY, w, h = box.astype('int')
                    # Use the center coordinates, width and height to get the coordinates of the top left corner
                    x = int(centerX - (w / 2))
                    y = int(centerY - (h / 2))
                    boxes.append([x, y, int(w), int(h)])
                    confidences.append(float(conf))
                    classIDs.append(classID)
        return boxes, confidences, classIDs

    def make_prediction(self, net, layer_names, labels, image, confidence, threshold):
        height, width = image.shape[:2]
        # Create a blob and pass it through the model
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(layer_names)
        # Extract bounding boxes, confidences and classIDs
        boxes, confidences, classIDs = self.extract_boxes_confidences_classids(outputs, confidence, width, height)
        # Apply Non-Max Suppression
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)
        return boxes, confidences, classIDs, idxs

    def draw_bounding_boxes(self, image, boxes, confidences, classIDs, idxs, colors, labels):
        if len(idxs) > 0:
            for i in idxs.flatten():
                # extract bounding box coordinates
                x, y = boxes[i][0], boxes[i][1]
                w, h = boxes[i][2], boxes[i][3]
                # draw the bounding box and label on the image
                color = [int(c) for c in colors[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(labels[classIDs[i]], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        # print(len(idxs))
        # print(len(classIDs))
        return image
if __name__ == '__main__':
    rospy.init_node('imageSegmenter', anonymous=True)
    # call constructor
    image_segmenter = imageSegmenter()


    # main loop
    rospy.spin()
