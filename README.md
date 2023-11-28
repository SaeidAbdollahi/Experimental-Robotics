# Marker Detection using RosBot2 and OpenCV

<p align="center">
<img src="resources/IMG_20231124_103921.jpg" width="400" height="400" />
</p>

The Rosbot2 robot has to follow environment, with the robot starting in (0, 0), and find four markers in the environment, with IDs 11, 12, 13, and 15.
The markers have the following meanings:
- Marker 11 -> rotate until you find marker 12; then reach marker 12
- Marker 12 -> rotate until you find marker 13; then reach marker 13
- Marker 13 -> rotate until you find marker 15; then reach marker 15
- Marker 15 -> done!


### Group Members: 
**Robotics Engineering, University of Genoa, Italy**
- SaeidAbdollahi 🆔 ([@Saeed](https://github.com/SaeidAbdollahi)) 
- Parinaz Ramezanpour 🆔 ([@ParinazRmp](https://github.com/ParinazRmp))
- Danial Sabzevari 🆔 ([@dssdanial](https://github.com/dssdanial)) 
  
## Table of Contents

1. [Introduction](#marker-detection-using-rosbot2-and-opencv)
    - [ROSbot 2](#rosbot-2)
    - [Robot Structure](#our-robot-structure)
    - [Code Structure](#code-structure)
2. [Robot Modelling URDF and XACRO](#1-robot-modelling-urdf-and-xacro)
    - [XACRO Arguments and Properties](#xacro-arguments-and-properties)
    - [Materials Definition](#materials-definition)
    - [Robot Description](#robot-description)
    - [Gazebo Plugin Integration](#gazebo-plugin-integration)

3. [Robot Logic (ROS Node)](#2-robot-logic-ros-node)
    - [Functionality](#functionality)

4. [ArUco Marker (Marker_Node)](#3-aruco-marker-marker_node)
    - [Functionality](#functionality-1)
    - [Implementation Details](#implementation-details)
    - [Integration with Robot Logic](#integration-with-robot-logic)

5. [Rviz-Gazebo Visualization](#3-rviz-gazebo-visualization)

6. [Practical Tests](#4-practical-tests-connecting-rosbot-to-your-wi-fi-network)
7. [Installation](#installation)
8. [Running the Code](#running-the-code)
9. [Possible Improvements](#possible-improvements)
10. [Troubleshooting](#troubleshooting)
11. [Authors & Contact Information](#authors--contact-information)
12. [License](#license)


#### ROSbot-2


###### source: https://husarion.com/tutorials/howtostart/rosbot---quick-start/


## Our-Robot-Structure

### Code-Structure
The pseudo code captures the structure and functionality of our algorithm, representing its various functions and logic flows within a more generalized syntax.

  
Robot Search Behavior |  Robot Control and Motion
:-------------------------:|:-------------------------:
![](https://github.com/SaeidAbdollahi/Experimental-Robotics/assets/32397445/158d130b-eeb6-4b92-b139-d521895bbde4) | ![](https://github.com/SaeidAbdollahi/Experimental-Robotics/assets/32397445/11d76ad2-175c-4e59-a3a4-91681ee814a1)



- **markerInfoCallback():** Callback function to receive marker information.

- **SearchMarker():** Function to search and locate the next marker.

- **AdjustRobotDirection():** Method to align the robot with the detected marker.

- **MoveRobotToMarker():** Function to move the robot towards the identified marker.

- **validateMarker():** Validates if the detected marker matches the expected marker.



</br>

### 1-Robot-modelling-URDF-and-XACRO

<p align="center">
<img src="https://github.com/SaeidAbdollahi/Experimental-Robotics/assets/32397445/8dde27b3-ed8e-45f0-9b52-86afc2e98ced" width="450" height="300" />
</p>

###### source: https://husarion.com/tutorials/howtostart/rosbot---quick-start/

- **XACRO-Arguments-and-Properties:**
    - `xacro:arg`: Defines arguments that can be passed to XACRO macros or files.
    - `xacro:property`: Sets properties with default values for reusability and modularity in defining robot components.

- **Materials-Definition:**
    
    **Materials:** 
    
    Define visual properties such as color and appearance for different components of the robot. 
    Materials are named and specified with RGBA values to represent colors.

- **Robot-Description:**

    - `robot`: Contains the robot model definition.
    - `name`: Specifies the name of the robot.
    - `xmlns:xacro`: Namespace declaration for using XACRO functionalities within the robot description.


- **Gazebo-Plugin-Integration:**

    - `gazebo`: Encloses Gazebo-specific plugins and configurations.
    - `plugin`: Configures Gazebo plugins for controlling the robot's movement and interaction with Gazebo simulation.
    Defines parameters such as update rate, wheel properties, joint names, topic names for velocity control, odometry, and covariance values for localization.






### 2-Robot-Logic-ROS-Node
This ROS node implements logic for a mobile robot to navigate through markers by utilizing marker information received from the /marker_info topic. The robot's goal is to locate specific markers, move towards them, and adjust its orientation to align with the markers.


### Functionality
The code orchestrates the following functionalities:

1. **Marker-Identification-and-Navigation-Marker-Identification:** It maintains a list of marker IDs to identify specific markers (markerList array).
- **Searching for Markers:** The robot rotates to locate the next marker by checking for marker IDs in the camera feed.
- **Validating Markers:** Once a marker is detected, it validates whether it matches the expected marker ID.
- **Moving to Marker:** Upon validation, the robot maneuvers towards the marker by adjusting its linear and angular speeds.
- **Adjusting Direction:** It adjusts its orientation to align with the marker for better accuracy.
  
2. **Control-and-Flags-Status:** 

- Several status flags (markerReached, reachAllowed, missionCompleted) control the robot's actions based on marker detection and navigation completion.



### 3-ArUco-Marker-Marker-Node

The ArUco Marker Publisher Node is responsible for detecting ArUco markers within incoming camera images and publishing marker information to the /marker_info topic. This node complements the robot logic by providing real-time marker detection and relevant marker data.

#### Functionality
- **ArUco Detection:** Utilizes the ArUco library for marker detection within the incoming camera stream.
- **Data Extraction:** Extracts marker information such as ID, size, and center coordinates from detected markers.
- **Image Publication:** Publishes the original image with detected markers overlaid (result topic) and the thresholded image for debugging purposes (debug topic).
- **ROS Integration:** Adheres to ROS standards by subscribing to the /image topic for camera feed and publishing marker information via the exp_assignment1::MarkerInfo message type.
Implementation Details
- **ArUco Detection Setup:** Uses the ArUco Marker Detector (aruco::MarkerDetector) to process incoming images and identify ArUco markers.
- **Image Processing:** Detects markers in the camera image, draws markers' outlines for visualization, and publishes the annotated image (result) and debug information (debug).
- **Marker Information Publication:** Publishes marker information including ID, size, and center coordinates on the /marker_info topic for consumption by other nodes.
Integration with Robot Logic.

The aruco_marker_publisher node seamlessly integrates with the robot logic by providing real-time marker information through the __exp_assignment1::MarkerInfo__ message. This data is crucial for the robot logic node to navigate and interact with specific markers in its environment.





### 3-Rviz-Gazebo-Simulation

### 4-Practical-tests-Connecting-ROSbot-to-network

Working with the real robot, you can connect to your Rosbot by following this procedure:
-connect to the network TP_LINK 
-login in ssh to husarion@<husarion_ip>    (192.168.1.xxx). 
- you can turn on the graphical interface as indicated in: https://husarion.com/tutorials/howtostart/rosbot---quick-start/

### Installation

#### Prerequisites-packages
- ROS Noetic /ROS2 installed
- vision_opencv
- Rosbot ROS (https://husarion.com/tutorials/howtostart/rosbot---quick-start/)
- aruco_ros (https://github.com/CarmineD8/aruco_ros) (https://github.com/pal-robotics/aruco_ros/tree/melodic-devel)
- robot_urdf (https://github.com/CarmineD8/robot_urdf)


### Running-the-Code

``` bash
 roslaunch XXXXXXXXXXXXXXXXXXXXXXXXXXX
```
In another terminal:
``` bash
roslaunch rosbot_bringup XXXXXXXXXXXXX

```
In another terminal:
``` bash
roslaunch rosbot_bringup XXXXXXXXXXXXX

```
In another terminal:
``` bash
roslaunch rosbot_bringup XXXXXXXXXXXXX

```

After having launched the simulation, we can manually run the node for marker detection:
```bash
rosrun aruco_ros marker_publisher /image:=/robot/camera1/image_raw
```
You can also give a look to the topic:
```bash
 /aruco_marker_publisher/result
 ```
 to check what is the effect of the ``aruco_marker_publisher``

### Possible-Improvements
- **Enhanced Marker Detection:** 
Implement advanced marker detection algorithms for faster and more accurate identification.

- **Path Planning:** Integrate path planning algorithms to optimize the robot's movement towards markers.




### Troubleshooting

ROS setup problems, potential errors in running the code, or common mistakes to avoid.

- 1-
- 2-
- 3-










### License
This project is licensed under the MIT License - see the LICENSE file for details.
