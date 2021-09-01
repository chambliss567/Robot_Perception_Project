## Overview 

This project was completed on the Construct platform, an online learning platform with dozens of courses teaching different fundamentals of ROS and robotics

This project sets out to fulfill the requirements defined in the **PhantomX Hexapod Perception Project**, part of the **ROS Perception** class. The project specifications and requirements are as follows

### Specifications: 
1. A PhantomCHexapod, created by TrossenRobotics, will be used. The transforms associated with this robot are defined in **hexapod_transforms.txt**
2. The robot's environment can be seen in [hexapod_environment.png](hexapod_environment.PNG). At of the circle locations along the green path are the following:
  - A stationary pink donut
  - A constantly rotating green alien holding a large gun
  - A stationary girl

### Requirements
1. The robot must be able to following the green path, either going left or right, depending on the direction published to the **/green_line_direction** topic
2. The robot must be able to stop at the star color that is published to the **/objective** topic
3. Using the **extended_object_detection** package, the **face detector** package, and the **face recognition** package, the robot will be able to do the following:
  - Use a simple HSV detector to detect the pink donute
  - Use a complex object to detect the rotating alien. This complex object will consist of two simple histogram detector objects for both the alien and the gun
  - Use the face detector and face recognition packages to detect and recognize the girl


## ROS Project Details

### Line Follower

**line_objective.py** start the **line_following_node** node. This node creates a **LineFollower** object, which handles moving the robot along the green line. Within the LineFollower Class, a **MovePhantomX** object is created, 
which publishes Twist messages to the **/phantomx/cmd_vel** topic. The LineFollower class detects the green line using the HSV detection found in the **cv2** package. Centroids are detected that correspond with green 
areas within the robot's immediate line of vision, and depending on the direction published to the **/green_line_direction** topic, the Twist message sent to the MovePhantomX object follows either the left-most or
right-most centroid. 

The LineFollower class also listens to the **/objective** topic to determine which color star to stop at, either blue, green, or red. Similar to following the green path, the cv2 HSV detection is used to detect centroids 
associated with each color. When the robot can only see the centroids associated with the star and not with the green line, the line_following_node is terminated.

### Object Detection

**hexapod_object_detection.launch** handles the detection of both the donut and the alien by calling the **extended_object_detection** node and package. The objects the node uses are defined in **Objects.xml**, which defines the HSV
detection parameters for the pink donut simple object and points to **alien_gun.yaml** and **alien.yaml** histogram files for both the alien's gun and the alien simple objects. For the alien, these two simple objects 
are combined into a complex object, where when the gun is to the right of the alien the robot detects the alien as one complex object. **hsv_color_params_collector.launch**, which calls the **hsv_color_params_collector_node**
of the extended_object_detection package, was used to determine the HSV values for the donut. **hist_color_params_collector_point.launch**, which calls the **hist_color_params_collector_point_node** of the extended_object_detection package, 
was used to generate the histograms for the alien and alien's gun. 

### Face Recognition

**face_recognition.launch** handles the detection and recognition of the girl's face. This launch file calls **face_detection.launc**, which uses the **face_detector** node and package to detect the presence of a face. 
face_recognition.launch additionally launches the **face_recognising_python_node**, defined in **recognize_face.py**. This file creates an object of class **FaceRecogniser**, which upon finding a face published to the 
**/face_detector/people_tracker_measurements_array** topic, uses the **face_recognition** package in conjuction with the **olive_hex.png** image to determine if the detected face is the girl, Olive. 

## Launching Nodes

Before starting this package, verify that the package is built and source with catkin

To run the line follower node, run the following:</br>
**rosrun my_hexapod_perception line_objective.py**

To launch the object detection, run the following:</br>
**roslaunch my_hexapod_perception hexapod_object_detection.launch**

To launch the face recogniser, run the following:</br>
**roslaunch my_hexapod_perception face_recognition.launch**