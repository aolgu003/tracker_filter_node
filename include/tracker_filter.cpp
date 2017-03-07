
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include "tracker_filter.h"

void trackerFilter::imageCallback( const sensor_msgs::ImageConstPtr& color_img )
{
	cv_bridge::CvImagePtr img_ptr;

  try 
  {
		img_ptr = cv_bridge::toCvCopy( color_img, sensor_msgs::image_encodings::BGR8 );
    image_ = img_ptr->image;
  } 
  catch ( cv_bridge::Exception& ex) 
  {
    ROS_ERROR("[TLD_TRACKER] Failed to convert image");
    return;
  }

  if ( trackerInitialized_ ) 
  {
    this->KFPrediction();
  }
  else 
  {
    tick_ = (double) cv::getTickCount();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void trackerFilter::trackerCallback( const sensor_msgs::RegionOfInterest& regionOfInterest )
{
  boundingBox_.x = regionOfInterest.x_offset;
  boundingBox_.y = regionOfInterest.y_offset;
  boundingBox_.width = regionOfInterest.width;
  boundingBox_.height = regionOfInterest.height;

  this->updateKF();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
void trackerFilter::KFPrediction()
{
  prevTick_ = (float) tick_;
  tick_ = (float) cv::getTickCount();

  dT_ = (tick_ - prevTick_) / (float) cv::getTickFrequency();

  // >>>> Matrix A
  //cv::setIdentity(kf_.transitionMatrix);
  kf_.transitionMatrix.at<float>(2) = dT_;
  kf_.transitionMatrix.at<float>(9) = dT_;
  
  state_ = kf_.predict();
  kf_.statePost = state_;
  // <<<< Matrix A


  predictedBoundingBox_.width = state_.at<float>(4);          
  predictedBoundingBox_.height = state_.at<float>(5);          
  predictedBoundingBox_.x = (float) (state_.at<float>(0) - predictedBoundingBox_.width / 2);          
  predictedBoundingBox_.y = (float) (state_.at<float>(1) - predictedBoundingBox_.height / 2);
      

  cv::Point center;          
  center.x = state_.at<float>(0);          
  center.y = state_.at<float>(1);          
  cv::circle(image_, center, 2, CV_RGB(255,0,0), -1);
  cv::rectangle(image_, predictedBoundingBox_, CV_RGB(255,0,0), 2);
  ROS_INFO("predicted center: %f, %f", state_.at<float>(0), state_.at<float>(1));

  ROS_INFO("Prediction dT: %f ", dT_);
      for (int i = 0; i < 36; i= i+6)
      {
          ROS_INFO("[ %f   %f   %f   %f  %f  %f ] ",kf_.transitionMatrix.at<float>(i),kf_.transitionMatrix.at<float>(i+1),kf_.transitionMatrix.at<float>(i+2),kf_.transitionMatrix.at<float>(i+3),kf_.transitionMatrix.at<float>(i+4),kf_.transitionMatrix.at<float>(i+5));
      }

      ROS_INFO("\n");
  cv::imshow("KF Result", image_);
  cv::waitKey(10);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
void trackerFilter::updateKF()
{
  meas_.at<float>(0) = (float) (boundingBox_.x + boundingBox_.width / 2);
  meas_.at<float>(1) = (float) (boundingBox_.y + boundingBox_.height / 2);
  meas_.at<float>(2) = (float) boundingBox_.width;
  meas_.at<float>(3) = (float) boundingBox_.height;

  if (!trackerInitialized_) // First detection!
  {

    // >>>> Initialization
    kf_.errorCovPre.at<float>(0) = 1.0; // px
    kf_.errorCovPre.at<float>(7) = 1.0; // px
    kf_.errorCovPre.at<float>(14) = 1.0;
    kf_.errorCovPre.at<float>(21) = 1.0;
    kf_.errorCovPre.at<float>(28) = 1.0; // px
    kf_.errorCovPre.at<float>(35) = 1.0; // px

    state_.at<float>(0) = meas_.at<float>(0);
    state_.at<float>(1) = meas_.at<float>(1);
    state_.at<float>(2) = 0;
    state_.at<float>(3) = 0;
    state_.at<float>(4) = meas_.at<float>(2);
    state_.at<float>(5) = meas_.at<float>(3);
    // <<<< Initialization
    ROS_INFO( "FILTER INITIALIZED" );
    kf_.statePost = state_;

    trackerInitialized_ = 1;

  }
  else 
  {
    ROS_INFO( "CORRECTION PERFORMED" );
    kf_.correct(meas_); // Kalman Correction
  }

  ROS_INFO("MEASUREMENT x: %f, MEASUREMENT y: %f", meas_.at<float>(0), meas_.at<float>(1));
  ROS_INFO("State x: %f, State y: %f", state_.at<float>(0), state_.at<float>(1));
  ROS_INFO("MeasSize: %d", measSize_);
  ROS_INFO( "EXITING KF UPDATE CALLBACK" );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool trackerFilter::isTrackerInitialized() 
{
  return trackerInitialized_;
}












