# Arm Project  
A ros package to control and simulate an autonomous robotic arm in picking and placing items.

## Simulation  
The robotic arm is a modified UR10 robotic arm  with a custom gripper and depth camera attachment. Simulations of the robot environment are done through Gazebo.

## Control  
A finite state machine was designed to create the control logic.  
![finite state machine](Screenshots/Designed_FSM_ArmProject.png)
The control was implemented using the Robot Operating System (ROS) in primarily Python, and some C++.  
Movement commands are sent through ros_control, and motion planning was done through the MoveIt API.  

## Perception  

### Object Shape and Coordinate Detection
From the depth camera attachment, point cloud data of the environment is obtained. The data is used to identify the location of all objects in the field. The algorithm is as follows:
1. Using the point cloud, the ground plane is removed
2. The remaining points are clustered via K-means clustering.  
3. Once clustered, a convex hull is constructed around each cluster, representing the objects.  
4. These clusters contain accurate coordinates of the objects in the field.
![pcl summary](Screenshots/pcl_segmenter_shot.png)


### Object Class Recognition
From the depth camera attachment, image data of the enviroment is obtained. The image data is used to identify all objects in the field, including determining the object classes and the general object coordinates. The algorithm is as follows:
1. YOLO v3 is used to classify each object in the field. 
2. OpenCV is then used to produce contours around each object.
3. For each object, output/determine the xyz coordinates of each point in the respective contour.
![pcl summary](Screenshots/image_segmenter_shot.png)

### Processing the Object Class and Positional Data
The result of the point cloud and image modules is a series of clusters, that contain different inforation. The point cloud clusters contain accurate coordinates of all suspected objects, and image clusters contain object classes and their respective coordinates (the coordinates are inaccurate). 

The clusters from the point cloud and image are then paired together, by relating the cluster coordinates. We use Munkres algorithm, with the Intersection over Union method used as the cost function. The result is clusters of each object with the respective positions and object class, both calculated with high accuracy.
