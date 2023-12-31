/*****************************
 Copyright 2011 Rafael Muñoz Salinas. All rights reserved.

 Redistribution and use in source and binary forms, with or without modification, are
 permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this list of
 conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice, this list
 of conditions and the following disclaimer in the documentation and/or other materials
 provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those of the
 authors and should not be interpreted as representing official policies, either expressed
 or implied, of Rafael Muñoz Salinas.
 ********************************/
/**
 * @file marker_publish.cpp
 * @author Bence Magyar
 * @date June 2014
 * @brief Modified copy of simple_single.cpp to publish all markers visible
 * (modified by Josh Langsfeld, 2014)
 */

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include <string> 
#include <cmath>
#include "exp_assignment1/MarkerInfo.h"


class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;

  double marker_size_;
  bool useCamInfo_;

  // Define some variables to descibe the detected marker's information and publish it
  int markerId;
  double markerSize;
  double markerCenterX;
  double markerCenterY;
  int markersCount;
  double imageRows;
  double imageCols;

   
  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;

  cv::Mat inImage_;
  
public:
  ArucoMarkerPublisher() :
      nh_("~"), it_(nh_), useCamInfo_(true), markerId(0), markersCount(0)
       
  {
    image_sub_ = it_.subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }
  
  int getMarkerId(){
     if(this->markersCount==0){
        return -1;
     }else{
        return this->markerId;
     }
  }
  
  //Set the marker's id 
  void setMarkerId(int id){
  	this->markerId = id;
  }
  
  //Set the markers count
  void setMarkersCount(int counts){
  	this->markersCount = counts;
  }

  //Get the marker's size 
  void setMarkerSize(aruco::Marker marker){
  	this->markerSize = marker.getArea(); 	
  }
  
  int getMarkerSize(){
     if(this->markersCount==0){
        return -1;
       }else{
        return (int)std::round(this->markerSize);  
     }
  }
  
  //Set the marker's center in the image
  void setMarkerCenter(aruco::Marker marker){
     cv::Point2f cent(0,0);
     cent = marker.getCenter();
     
     this->markerCenterX = cent.x;
     this->markerCenterY = cent.y;
  }

  //Get the marker's center in the image
   double getMarkerCenterX(){
     return this->markerCenterX;
  }
  
  //Get the marker's center in the image
   double getMarkerCenterY(){
     return this->markerCenterY;
  }
  
  //Set the image's size
  void setImageSize(cv_bridge::CvImagePtr cv_ptr){
     this->imageRows  = cv_ptr->image.rows;
     this->imageCols = cv_ptr->image.cols;
  }
  
  //Get the image's size
  double getImageRows(){
     return this->imageRows;
  }
  
  //Get the image's size
  double getImageCols(){
     return this->imageCols;
  }
  

  void image_callback(const sensor_msgs::ImageConstPtr& msg)
  {
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;
      this->setImageSize(cv_ptr);
      
   
      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
      
      this->setMarkersCount(markers_.size());
      std::cout << "The id of the detected marker detected is: ";
		
      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        std::cout << markers_.at(i).id << " ";
        this->setMarkerId(markers_.at(i).id);
        this->setMarkerSize(markers_.at(i));
        this->setMarkerCenter(markers_.at(i));
      }
      std::cout << std::endl;
        
      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");
  ArucoMarkerPublisher node;
  
  ros::NodeHandle n;
  ros::Publisher markerInfo_pub = n.advertise<exp_assignment1::MarkerInfo>("marker_info", 1000);
  
  while(ros::ok())
  {
    exp_assignment1::MarkerInfo mf;
    mf.marker_id = node.getMarkerId();
    mf.marker_size = node.getMarkerSize();
    mf.marker_center_x = node.getMarkerCenterX();
    mf.marker_center_y = node.getMarkerCenterY();;
    mf.markersCount = 0;
    mf.imageRows = node.getImageRows();
    mf.imageCols = node.getImageCols();
    markerInfo_pub.publish(mf);
    
    ros::spinOnce();
  }
  return 0;
}
