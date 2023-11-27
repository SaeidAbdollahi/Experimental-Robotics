#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <string> 
#include "exp_assignment1/MarkerInfo.h"
#include <cmath>

//Markers Id 
int markerList[4] = {11, 12, 13, 15};
int markerId = 0;
int currentMarker;
int lastDetectedMarker_Direction = 1;

//Status Falgs
int markerReached = 0;
int reachAllowed = 0;
int missionCompleted = 0;

//Control Threshold
const double alpha_distance = 0.1;
const double alpha_missalign = 0.01;
const double sizeThreshold = 20000;
const double missAlignmentThreshold = 150;
int speedResolution = 0;
double searchSpeed[4] = {5, 2.5, 1, 0.5};

//Marker Information
exp_assignment1::MarkerInfo mf;
void markerInfoCallback(exp_assignment1::MarkerInfo msg);
int validateMarker(int detectedMarker);

//Robot Behaviours
void RobotStop();
void RobotTurn(double speed, int direction);
void RobotMoveForward(double speed, int direction);
double linearSpeed;
ros::Publisher vel_pub;
geometry_msgs::Twist msg;

//Control Tasks
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
  
  ros::Rate rate(10);
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
       std::cout<<"Mission Completed!"<<std::endl;
       RobotStop();
    }
    vel_pub.publish(msg);
    
    ros::spinOnce();
    rate.sleep();
   }
    return 0;
}


void markerInfoCallback(exp_assignment1::MarkerInfo msg){
  mf = msg;
}

void RobotStop(){
   msg.linear.x = 0;
   msg.linear.y = 0;
   msg.angular.z = 0;
}

void RobotSpeedDown(){
   msg.linear.x = 0.25;
   msg.linear.y = 0;
   msg.angular.z = 0;
}

double RobotLinearSpeedUp(double targetSpeed);
double RobotAngularSpeedUp(double targetSpeed);

void RobotTurn(double speed, int direction){
   msg.linear.x = 0;
   msg.linear.y = 0;
   msg.angular.z = direction * speed;
}

void RobotMoveForward(double speed, int direction){
   msg.linear.x = direction * speed;
   msg.linear.y = 0;
   msg.angular.z = 0;
}

void SearchMarker(){
   int detectedMarker = mf.marker_id;
      if(detectedMarker<0){
          std::cout << "marker not found" << std::endl;
         //If no marker found, continue the search 
         RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
      }else{
         //If a marker found, validate the marker
         int isMarkerValid = validateMarker(detectedMarker);
         if(isMarkerValid){
           if(detectedMarker == currentMarker){
             //If the detected marker is valid and it is equal to curent marker, finish the search
             reachAllowed = 1;
	     std::cout << "I am moving to the marker" << std::endl;
            }else{
             std::cout << "I lost the marker again..." << std::endl;
             //If the marker is valid but it is not equal to curent marker, do the search
             RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
            }
         }else{
            std::cout << "The marker is not valid" << std::endl;
           //If the detected marker is not a valid markers
           //Try to change the robot position w.r to the marker, so the robot can check the marker from another viewpoint
            double miss_alignX = mf.marker_center_x - mf.imageRows/2.0;
            if(miss_alignX>0){
               RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
               RobotMoveForward(searchSpeed[3], -1);
            }else{
               RobotTurn(searchSpeed[3], lastDetectedMarker_Direction);
               RobotMoveForward(searchSpeed[3], -1);
            }
         }
      }
 }
  
 int validateMarker(int detectedMarker){
    int isMarkerValid=0;
    for(int index=0;index<sizeof(markerList);index++){
       if(detectedMarker == markerList[index]){
         isMarkerValid = 1;
         break;
       }
    }
    return isMarkerValid;
 }

void AdjustRobotDirection(){
   //Compute the robot miss alignment w.r to the current marker
   double miss_alignX = mf.marker_center_x - mf.imageRows/2.0;
   if(std::fabs(miss_alignX) > missAlignmentThreshold){
std::cout << "I am adjusting...." << std::endl;
      //The robot is not aligned, rotate the robot to make it align enough
      double angular_speed = RobotAngularSpeedUp(std::fabs(alpha_missalign * miss_alignX));

      if(miss_alignX>0){
        lastDetectedMarker_Direction = -1;
        RobotTurn(angular_speed, lastDetectedMarker_Direction);
      }else{
        lastDetectedMarker_Direction = 1;
        RobotTurn(angular_speed, lastDetectedMarker_Direction);
      } 
   }else{
std::cout << "Adjustment finished" << std::endl;
     //Robot is aligned w.r to the marker
     RobotSpeedDown();
   }
}

void MoveRobotToMarker(){
    
   //Compute the robot distance from the current marker
   double distance_error = sizeThreshold/mf.marker_size;
   //std::cout << distance_error << std::endl;
   //The robot is far from the marker, move the robot to make it close enough
   if(distance_error>=1){
      double speed = RobotLinearSpeedUp(alpha_distance * distance_error);
      std::cout << "I am getting closer" << std::endl;
      RobotMoveForward(speed, -1);
   }else if(distance_error<1 && distance_error>0){
std::cout << "I am close enough..." << std::endl;
     //Robot is close enogh to the marker
     RobotStop();
     //Update the current marker
     markerId = markerId + 1;
     std::cout<<markerId<<std::endl;
     if(markerId > 3){
        std::cout<<"mission complete"<<std::endl;
        missionCompleted = 1;
     }
     reachAllowed = 0;
   }else{
    //Robot lost the marker
     reachAllowed = 0;
   }
}

double RobotLinearSpeedUp(double targetSpeed){
  if(targetSpeed>0.5){
    return 0.5;
  }else{
    return targetSpeed;
  }
}
double RobotAngularSpeedUp(double targetSpeed){
  if(targetSpeed>0.5){
    return 0.5;
  }else{
    return targetSpeed;
  }  
}

