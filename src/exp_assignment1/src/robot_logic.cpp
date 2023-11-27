#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <string> 
#include "exp_assignment1/MarkerInfo.h"
#include <cmath>

//Global variables
ros::Publisher vel_pub;
geometry_msgs::Twist msg;

//Markers Id 
int markerList[4] = {11, 12, 13, 15};
int markerId = 0;
int currentMarker;
int lastDetectedMarker_Direction = 1;

//Marker Information
exp_assignment1::MarkerInfo mf;
void markerInfoCallback(exp_assignment1::MarkerInfo msg);
int validateMarker(int detectedMarker);

//Status Falgs
int markerReached = 0;
int reachAllowed = 0;
int missionCompleted = 0;

//Control Threshold
const double alpha_distance = 0.1;
const double alpha_missalign = 0.01;
const double sizeThreshold = 16500;
const double missAlignmentThreshold = 100;
double searchSpeed[4] = {5, 2.5, 1, 0.5};

//Simple Robot's Behaviours
void RobotStop();
void RobotTurn(double speed, int direction);
void RobotMoveForward(double speed, int direction);
double RobotLinearSpeedUp(double targetSpeed);
double RobotAngularSpeedUp(double targetSpeed);
void RobotSpeedDown();
double linearSpeed;

//Complex Robot's Behaviours
void SearchMarker();
void AdjustRobotDirection();
void MoveRobotToMarker();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_logic");
  ros::NodeHandle n;
  
  //Create velocity publisher
  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  //Create marker subscriber
  ros::Subscriber markerInfo_sub = n.subscribe("/marker_info", 1000, markerInfoCallback);
  
  //Adjust the frequency of the node
  ros::Rate rate(10);

  //Main Logic-Loop
  while(ros::ok()){
    //Try to find all markers and finish the mission!
    if(!missionCompleted){
       // Try to find the next marker!
       currentMarker = markerList[markerId];
       //-----------Try to find the Marker-----------
       //Step1-Rotate robot to find the current market
       SearchMarker();
       //-----------Marker Found, now try to reach the marker-----------
       if(reachAllowed){
          //Step1-Move the robot to the marker
          MoveRobotToMarker();
          //Step2-Adjust the robot direction
          AdjustRobotDirection();
       }
    }else{
       //All markers found and mission is completed!
       std::cout<<"All the markers found, Mission Completed!"<<std::endl;
       RobotStop();
    }
    vel_pub.publish(msg);
    
    ros::spinOnce();
    rate.sleep();
   }
    return 0;
}
//----------------------------------Complex Robot's Behaviours----------------------------------
//1-Robot's Search Behavior
void SearchMarker(){
   //Read the current marker from the sensor
   int detectedMarker = mf.marker_id;
      if(detectedMarker<0){
          std::cout << "Marker not found! Searching..." << std::endl;
         //If no marker found, continue the search 
         RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
      }else{
         //If a marker found, validate the marker
         int isMarkerValid = validateMarker(detectedMarker);
         if(isMarkerValid){
            //If the detected marker is valid and it is equal to curent marker, finish the search
           if(detectedMarker == currentMarker){
             std::cout << "Marker found!" << std::endl;
             reachAllowed = 1;
            }else{
             std::cout << "A Market found, but it is not the target marker! Searching..." << std::endl;
             //If the marker is valid but it is not equal to curent marker, do the search
             RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
            }
         }else{
            std::cout << "The detected marker is not valid!" << std::endl;
           //If the detected marker is not a valid markers
           //Try to change the robot position w.r to the marker, so the robot can check the marker from another viewpoint
            double miss_alignX = mf.marker_center_x - mf.imageRows/2.0;
            if(miss_alignX>0){
               RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
               RobotMoveForward(searchSpeed[3], 1);
            }else{
               RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
               RobotMoveForward(searchSpeed[3], 1);
            }
         }
      }
 }

//2-Robot's Adjusting_w.r_to_the_Marker Behavior
void AdjustRobotDirection(){
   std::cout << "Adjusting..." << std::endl;
   //Compute the robot miss alignment w.r to the current marker
   double miss_alignX = mf.marker_center_x - mf.imageRows/2.0;
   //The robot is not aligned, rotate the robot to make it align enough
   if(std::fabs(miss_alignX) > missAlignmentThreshold){
      //Set the alignemt speed
      double angular_speed = RobotAngularSpeedUp(std::fabs(alpha_missalign * miss_alignX));
      //If the marker is in the right, rotate right
      if(miss_alignX>0){
        RobotTurn(angular_speed, lastDetectedMarker_Direction);
        //Keep in memory the direction that the robot could find a marker
        lastDetectedMarker_Direction = -1;
      }else{
        //If the marker is in the left, rotate left
        RobotTurn(angular_speed, lastDetectedMarker_Direction);
        //Keep in memory the direction that the robot could find a marker
        lastDetectedMarker_Direction = 1;
      } 
   }else{
      std::cout << "Adjustment finished!" << std::endl;
     //Robot is aligned w.r to the marker
     RobotSpeedDown();
   }
}

//R3-obot's Move_to_the_Marker Behavior
void MoveRobotToMarker(){
   //Compute the robot distance from the current marker
   double distance_error = sizeThreshold/mf.marker_size;
   //The robot is far from the marker, move the robot to make it close enough
   if(distance_error>=1){
      std::cout << "I am getting closer to the marker!" << std::endl;
      //Adjust the robot's speed
      double speed = RobotLinearSpeedUp(alpha_distance * distance_error);
      //Move robot forward
      RobotMoveForward(speed, 1);
   }else if(distance_error<1 && distance_error>0){
std::cout << "I am close enough..." << std::endl;
     //Robot is close enogh to the marker, stop the robot
     RobotStop();
     //Update the marker id
     std::cout<<"Marker with id "<< markerId<<"is found!"<<std::endl;
     markerId = markerId + 1;
     //Check if the mission is completed or not
     if(markerId > 3){
        missionCompleted = 1;
     }else{
        std::cout<<"Try to find the next marker with id "<< markerId<<std::endl;
     }
     reachAllowed = 0;
   }else{
      //The reported DISTANCE_ERROR is invalid because the robot lost the marker
		//Stop moving the robot to the marker
      reachAllowed = 0;
   }
}

//----------------------------------Simple Robot's Behaviours----------------------------------

//Stop the robot
void RobotStop(){
   msg.linear.x = 0;
   msg.linear.y = 0;
   msg.angular.z = 0;
}

//Turn the robot
void RobotTurn(double speed, int direction){
   msg.linear.x = 0;
   msg.linear.y = 0;
   msg.angular.z = direction * speed;
}

//Move the robot forward
void RobotMoveForward(double speed, int direction){
   msg.linear.x = direction * speed;
   msg.linear.y = 0;
   msg.angular.z = 0;
}

//Speed regulator
double RobotLinearSpeedUp(double targetSpeed){
  if(targetSpeed>0.5){
    return 0.5;
  }else{
    return targetSpeed;
  }
}

//Speed regulator
double RobotAngularSpeedUp(double targetSpeed){
  if(targetSpeed>0.5){
    return 0.5;
  }else{
    return targetSpeed;
  }  
}

//Speed down the robot
void RobotSpeedDown(){
   msg.linear.x = 0.25;
   msg.linear.y = 0;
   msg.angular.z = 0;
}

//----------------------------------Markers Operation----------------------------------
 //Validating the marker 
int validateMarker(int detectedMarker){
   std::cout << "Validating the marker..." << std::endl;
    int isMarkerValid=0;
    //The detected marker is valid if it is in the markers list
    for(int index=0;index<sizeof(markerList);index++){
       if(detectedMarker == markerList[index]){
         isMarkerValid = 1;
         break;
       }
    }
    return isMarkerValid;
 }

//Set the marker's information
 void markerInfoCallback(exp_assignment1::MarkerInfo msg){
  mf = msg;
}
