




### ROSbot 2
![Alt text](image.png)

###### source: https://husarion.com/tutorials/howtostart/rosbot---quick-start/

### Robot Logic ROS Node
This ROS node implements logic for a mobile robot to navigate through markers by utilizing marker information received from the /marker_info topic. The robot's goal is to locate specific markers, move towards them, and adjust its orientation to align with the markers.

#### Functionality
The code orchestrates the following functionalities:

1. **Marker Identification and Navigation Marker Identification:**
It maintains a list of marker IDs to identify specific markers (markerList array).
Searching for Markers: The robot rotates to locate the next marker by checking for marker IDs in the camera feed.
Validating Markers: Once a marker is detected, it validates whether it matches the expected marker ID.
Moving to Marker: Upon validation, the robot maneuvers towards the marker by adjusting its linear and angular speeds.
Adjusting Direction: It adjusts its orientation to align with the marker for better accuracy.
2. **Control and Status Flags Status Flags:** 

    Several status flags (markerReached, reachAllowed, missionCompleted) control the robot's actions based on marker detection and navigation completion.


### Installation

#### Prerequisites packages
- ROS Noetic /ROS2 installed
- vision_opencv
- Rosbot ROS
- aruco_ros (https://github.com/CarmineD8/aruco_ros)
- robot_urdf (https://github.com/CarmineD8/robot_urdf)


### Running the Code



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


### Authors

- SaeidAbdollahi (@Saeed)
- Parinaz Ramezanpour (@ParinazRmp)
- Danial Sabzevari (@dssdanial)



### License
This project is licensed under the MIT License - see the LICENSE file for details.
