#! /usr/bin/env python

import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from control.msg import image_points
from control.srv import doService
import cv2
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
        layer_names = self.net.getLayerNames()
        self.layer_names = [layer_names[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]
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
        self.xyz = ros_numpy.point_cloud2.get_xyz_points(ros_cloud_arr, remove_nans=False)
        rgb = ros_numpy.point_cloud2.split_rgb_field(ros_cloud_arr)
        img = np.array([rgb['r'],rgb['g'],rgb['b']]) #shape is 3,480,640
        img = np.moveaxis(img, 0, 2) #shape is 480,640,3
        self.img = img
        self.height, self.width = self.img.shape[:2]
        self.yolo()
        self.execute = True #this must be false
        print("Finished execution callback")

    def yolo(self):
        self.boxes, self.confidences, self.classIDs, display_boxes = self.make_prediction(
            self.net, self.layer_names, self.labels, self.img, self.confidence, self.threshold)
        self.classes = [self.labels[cid] for cid in self.classIDs]
        self.image_segments = self.make_contour_hulls()
        # --- for bounding box 
        #   extract the image
        #   get contours in image
        #   get convex contour hull in image
        #   shrink the contour hull in image
        # return a list of convex hulls ----
        # --- turn each contour hull into a xyz coordinate ---
        # publish that list of convex hulls (same size/indexing as boxes)
        
        self.pub_predictions(self.boxes,self.classes)
        ### The code below is image publishing code, it can be removed
        colors = np.random.randint(0, 255, size=(len(self.labels), 3), dtype='uint8')
        imageBounded = self.draw_bounding_boxes(self.img, display_boxes, self.confidences, self.classIDs, colors, self.labels)
        self.image_pub.publish(
            ros_numpy.image.numpy_to_image(
                imageBounded, "rgb8"))

    def make_contour_hulls(self):
        hulls = []
        for box in self.boxes:
            centerX,centerY,w,h = box[0],box[1],box[2],box[3]
            top_left = self.pixle_to_xy((centerX - (w//2) , centerY - (h//2)))
            top_right = self.pixle_to_xy((centerX + (w//2), centerY - (h//2)))
            bot_left = self.pixle_to_xy((centerX - (w//2) , centerY + (h//2)))
            bot_right = self.pixle_to_xy((centerX + (w//2), centerY + (h//2)))
        return hulls

    def subset_detections(self, l , indexes):
        return [l[i] for i in indexes]
    def validate_pixel(self, coord):
        x = int(coord[0])
        col_ind = min(x,self.width-1)
        col_ind = max(col_ind,0)
        y = int(coord[1])
        row_ind = min(y,self.height-1)
        row_ind = max(row_ind,0)
        return row_ind, col_ind
    def pixle_to_xy(self, coord):
        row_ind, col_ind = self.validate_pixel(coord)
        return self.xyz[row_ind][col_ind][0],self.xyz[row_ind][col_ind][1]
    def get_coordinates(self,box):
        #converts box pixle coordinates to the x,y  coordinates
        centerX,centerY,w,h = box[0],box[1],box[2],box[3]
        top_left = self.pixle_to_xy((centerX - (w//2) , centerY - (h//2)))
        top_right = self.pixle_to_xy((centerX + (w//2), centerY - (h//2)))
        bot_left = self.pixle_to_xy((centerX - (w//2) , centerY + (h//2)))
        bot_right = self.pixle_to_xy((centerX + (w//2), centerY + (h//2)))
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

    def extract_boxes_confidences_classids(self,outputs, confidence, width, height):
        # Helper for make_predictions
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
                    print 'Detection: ',detection[0:4]
                    box = detection[0:4] * np.array([width, height, width, height])
                    centerX, centerY, w, h = box.astype('int')
                    boxes.append([centerX, centerY, w, h])
                    print 'width ',width, 'height', height
                    print 'Box: ', boxes[-1]
                    # Use the center coordinates, width and height to get the coordinates of the top left corner
                    x = int(centerX - (w / 2))
                    y = int(centerY - (h / 2))
                    display_boxes.append([x, y, int(w), int(h)])
                    confidences.append(float(conf))
                    classIDs.append(classID)
        return boxes, confidences, classIDs, display_boxes

    def make_prediction(self, net, layer_names, labels, image, confidence, threshold):
        height, width = image.shape[:2]
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outputs = net.forward(layer_names)
        boxes, confidences, classIDs, display_boxes = self.extract_boxes_confidences_classids(outputs, confidence, width, height)
        # If requested, further combine similar bounding boxes using NMS
        if self.NMS:
            idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)
            if len(idxs)>0:
                idxs = idxs.flatten()
                boxes = self.subset_detections(boxes, idxs)
                display_boxes = self.subset_detections(display_boxes, idxs)
                confidences = self.subset_detections(confidences, idxs)
                classIDs = self.subset_detections(classIDs, idxs)
        return boxes, confidences, classIDs, display_boxes

    def draw_bounding_boxes(self, image, boxes, confidences, classIDs, colors, labels):
        # In order for following cv2 to operate on an image it must UMat,
        # but it is converted back into numpy array later on. See https://github.com/opencv/opencv/issues/14866
        image = cv2.UMat(image)
        for i in range(len(boxes)):
            x, y = boxes[i][0], boxes[i][1]
            w, h = boxes[i][2], boxes[i][3]
            # draw the bounding box and label on the image
            color = [int(c) for c in colors[classIDs[i]]]
            cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
            text = "{}: {:.4f}".format(labels[classIDs[i]], confidences[i])
            cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        image = image.get()
        return image

if __name__ == '__main__':
    rospy.init_node('image_segmenter', anonymous=True)
    image_segmenter = imageSegmenter()
    # main loop
    rospy.spin()
