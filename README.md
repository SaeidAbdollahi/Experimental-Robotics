# Marker Detection using RosBot2 and OpenCV




#### ROSbot 2
<p align="center">

<img src="https://github.com/SaeidAbdollahi/Experimental-Robotics/assets/32397445/b96396fc-8b48-4089-89c1-2e8954a8d37c" width="350" height="300" />
</p>

###### source: https://husarion.com/tutorials/howtostart/rosbot---quick-start/


## Our Robot Structure

### 1- Robot modelling URDF and XACRO

<p align="center">
<img src="https://github.com/SaeidAbdollahi/Experimental-Robotics/assets/32397445/8dde27b3-ed8e-45f0-9b52-86afc2e98ced" width="450" height="300" />
</p>

###### source: https://husarion.com/tutorials/howtostart/rosbot---quick-start/

- **XACRO Arguments and Properties:**
    - `xacro:arg`: Defines arguments that can be passed to XACRO macros or files.
    - `xacro:property`: Sets properties with default values for reusability and modularity in defining robot components.

- **Materials Definition:**
    
    **Materials:** 
    
    Define visual properties such as color and appearance for different components of the robot. 
    Materials are named and specified with RGBA values to represent colors.

- **Robot Description:**

    - `robot`: Contains the robot model definition.
    - `name`: Specifies the name of the robot.
    - `xmlns:xacro`: Namespace declaration for using XACRO functionalities within the robot description.


- **Gazebo Plugin Integration:**

    - `gazebo`: Encloses Gazebo-specific plugins and configurations.
    - `plugin`: Configures Gazebo plugins for controlling the robot's movement and interaction with Gazebo simulation.
    Defines parameters such as update rate, wheel properties, joint names, topic names for velocity control, odometry, and covariance values for localization.






### 2- Robot Logic (ROS Node)
This ROS node implements logic for a mobile robot to navigate through markers by utilizing marker information received from the /marker_info topic. The robot's goal is to locate specific markers, move towards them, and adjust its orientation to align with the markers.


### Functionality
The code orchestrates the following functionalities:

1. **Marker Identification and Navigation Marker Identification:** It maintains a list of marker IDs to identify specific markers (markerList array).
- **Searching for Markers:** The robot rotates to locate the next marker by checking for marker IDs in the camera feed.
- **Validating Markers:** Once a marker is detected, it validates whether it matches the expected marker ID.
- **Moving to Marker:** Upon validation, the robot maneuvers towards the marker by adjusting its linear and angular speeds.
- **Adjusting Direction:** It adjusts its orientation to align with the marker for better accuracy.
  
2. **Control and Status Flags Status Flags:** 

- Several status flags (markerReached, reachAllowed, missionCompleted) control the robot's actions based on marker detection and navigation completion.



### 3- ArUco Marker (Marker_Node)

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





### 3- Rviz-Gazebo Visualization

### 4- Practical tests (Connecting ROSbot to your Wi-Fi network)


### Installation

#### Prerequisites packages
- ROS Noetic /ROS2 installed
- vision_opencv
- Rosbot ROS (https://husarion.com/tutorials/howtostart/rosbot---quick-start/)
- aruco_ros (https://github.com/CarmineD8/aruco_ros) (https://github.com/pal-robotics/aruco_ros/tree/melodic-devel)
- robot_urdf (https://github.com/CarmineD8/robot_urdf)


### Running the Code



After having launched the simulation, we can manually run the node for marker detection:
```bash
rosrun aruco_ros marker_publisher /image:=/robot/camera1/image_raw
```
You can also give a look to the topic:
```bash
 /aruco_marker_publisher/result
 ```
 to check what is the effect of the ``aruco_marker_publisher``

### Possible Improvements
- **Enhanced Marker Detection:** 
Implement advanced marker detection algorithms for faster and more accurate identification.

- **Path Planning:** Integrate path planning algorithms to optimize the robot's movement towards markers.


### Code Structure

- **markerInfoCallback():** Callback function to receive marker information.

- **SearchMarker():** Function to search and locate the next marker.

- **AdjustRobotDirection():** Method to align the robot with the detected marker.


- **MoveRobotToMarker():** Function to move the robot towards the identified marker.

- **validateMarker():** Validates if the detected marker matches the expected marker.



### Troubleshooting

ROS setup problems, potential errors in running the code, or common mistakes to avoid.

- 1-
- 2-
- 3-

### Experimental Testing

Detail abouts the testing used to validate the functionality of the code. 





### Authors & Contact Information

- SaeidAbdollahi (@Saeed) :email:
- Parinaz Ramezanpour (@ParinazRmp) :email:
- Danial Sabzevari (@dssdanial) :email:



### License
This project is licensed under the MIT License - see the LICENSE file for details.
