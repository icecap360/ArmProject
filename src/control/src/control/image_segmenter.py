#! /usr/bin/env python

import rospy
from control.msg import class_list, arm_parameters
from control.srv import isGo
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from control.msg import image_points
import cv2
# import os
# import time
import rospkg

class imageSegmenter:
    def __init__(self):
        rospack = rospkg.RosPack()
        dir = rospack.get_path('control') + "/src/control/"
        self.weight_path= dir + "models/yolov3.weights"
        self.config_path= dir + "models/yolov3.cfg"
        self.labels_path= dir + "models/coco.names"
        self.confidence = 0.5
        self.threshold = 0.3
        self.NMS = True
        self.net = cv2.dnn.readNetFromDarknet(self.config_path, self.weight_path)
        layer_names = self.net.getLayerNames()
        self.layer_names = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
        self.labels = open(self.labels_path).read().strip().split('\n')
        # All SERVICES and TOPICS MUST be created BELOW
        self.sub = rospy.Subscriber('/camera/depth/points', PointCloud2, self.saveImg)
        self.execute =True
        self.pub = rospy.Publisher('image_segmenter', image_points, queue_size=1)

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
        self.execute = True #this must be false

    def yolo(self):
        self.img = cv2.resize(self.img,(416,416))#reshape to 416*416 as per blob
        #self.img = self.rotate_image(self.img, 270)
        self.boxes, self.confidences, self.classIDs, display_boxes = self.make_prediction(
            self.net, self.layer_names, self.labels, self.img, self.confidence, self.threshold)
        self.classes = [self.labels[cid] for cid in self.classIDs]
        print('Boxes',self.boxes)
        self.pub_predictions(self.boxes,self.classes)
        ### The code below is image printing code, it should be removed in the future
        colors = np.random.randint(0, 255, size=(len(self.labels), 3), dtype='uint8')
        imageBounded = self.draw_bounding_boxes(self.img, display_boxes, self.confidences, self.classIDs, colors, self.labels)
        #cv2.imshow('YOLO Object Detection', self.img)
        self.image_pub.publish(
            ros_numpy.image.numpy_to_image(
                imageBounded, "rgb8"))

    # def rotate_image(self, image, angle):
    #     image_center = tuple(np.array(image.shape[1::-1]) / 2)
    #     rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    #     result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    #     return result
    def get_coordinates(self,box):
        #converts box pixle coordinates to the x,y  coordinates
        centerX,centerY,w,h = box[0],box[1],box[2],box[3]
        top_left = (centerX - (w/2) , centerY - (h/2))
        top_right = (centerX + (w/2), centerY - (h/2))
        bot_left = (centerX - (w/2) , centerY + (h/2))
        bot_right = (centerX + (w/2), centerY + (h/2))
        print('Raw Box')
        print(top_left)
        print(top_right)
        print(bot_left)
        print(bot_right )
        return (top_left,top_right,bot_left,bot_right)
    def pub_predictions(self, det_boxes, det_classes):
        msg = image_points()
        msg.x,msg.y,msg.obj_class = [],[],[]
        end_of_det = -100 #this should come from the param server
        for i in range(len(det_boxes)):
            coords = self.get_coordinates(det_boxes[i])
            for coord in coords:
                msg.x.append(coord[0])
                msg.y.append(coord[1])
            print(type(det_classes[i]))
            msg.obj_class.append(det_classes[i])
            #mark the end of detection
            msg.x.append(end_of_det)
            msg.y.append(end_of_det)
            msg.obj_class.append(end_of_det)
        self.pub.publish(msg.x, msg.y, str(msg.obj_class))

    def extract_boxes_confidences_classids(self,outputs, confidence, width, height):
        boxes = []
        confidences = []
        classIDs = []
        display_boxes = []
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
                    boxes.append([centerX, centerY, w, h])
                    # Use the center coordinates, width and height to get the coordinates of the top left corner
                    x = int(centerX - (w / 2))
                    y = int(centerY - (h / 2))
                    display_boxes.append([x, y, int(w), int(h)])
                    confidences.append(float(conf))
                    classIDs.append(classID)
        return boxes, confidences, classIDs, display_boxes

    def make_prediction(self, net, layer_names, labels, image, confidence, threshold):
        height, width = image.shape[:2]
        # Create a blob and pass it through the model
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(layer_names)
        # Extract bounding boxes, confidences and classIDs
        boxes, confidences, classIDs, display_boxes = self.extract_boxes_confidences_classids(outputs, confidence, width, height)
        # Apply Non-Max Suppression
        if self.NMS:
            idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)
            if len(idxs)>0:
                idxs = idxs.flatten()
                boxes = [boxes[i] for i in idxs]
                display_boxes = [display_boxes[i] for i in idxs]
                confidences = [confidences[i] for i in idxs]
                classIDs = [classIDs[i] for i in idxs]
        return boxes, confidences, classIDs, display_boxes

    def draw_bounding_boxes(self, image, boxes, confidences, classIDs, colors, labels):
        for i in range(len(boxes)):
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
