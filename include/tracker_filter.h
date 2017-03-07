#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>


#include <opencv/cv.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>


class trackerFilter 
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber subImg_;
  ros::Subscriber subROI_;
  ros::Publisher pub_;

  cv::KalmanFilter kf_;
  cv::Mat state_;
  cv::Mat meas_;
  cv::Mat procNoise_;

  cv::Mat image_;
  cv::Mat templateImage_;
  cv::Rect2d boundingBox_;
  cv::Rect2d predictedBoundingBox_;

  //Flags
  bool trackerInitialized_;
  bool firstTrack_;

  int stateSize_;
  int measSize_;
  int contrSize_;
  unsigned int type_;

  float prevTick_;
  float tick_;
  float dT_;

  public:
    trackerFilter() : 
      it_(nh_), 
      trackerInitialized_(false),
      firstTrack_(false),
      stateSize_(6),
      measSize_(4),
      contrSize_(0),
      type_(CV_32F),
      prevTick_(),
      dT_(0)  
    {
      state_ = cv::Mat_<float>(stateSize_,1);
      meas_ = cv::Mat_<float>(measSize_,1);

      ROS_INFO( "ENTERING CONSTRUCTOR");
      subImg_ = it_.subscribe( "cv_camera/image_raw", 1, &trackerFilter::imageCallback, this );
      subROI_ = nh_.subscribe( "object_Tracker/roi", 1, &trackerFilter::trackerCallback, this );
      ROS_INFO( "SUBBED" );
      kf_ = cv::KalmanFilter(stateSize_, measSize_, contrSize_, type_);
      cv::setIdentity(kf_.transitionMatrix);

      kf_.measurementMatrix = cv::Mat::zeros(measSize_, stateSize_, type_);
      kf_.measurementMatrix.at<float>(0) = 1.0f;
      kf_.measurementMatrix.at<float>(7) = 1.0f;
      kf_.measurementMatrix.at<float>(16) = 1.0f;
      kf_.measurementMatrix.at<float>(23) = 1.0f;

      //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));  
      kf_.processNoiseCov.at<float>(0) = 1e-2;
      kf_.processNoiseCov.at<float>(7) = 1e-2;
      kf_.processNoiseCov.at<float>(14) = 1e-1;
      kf_.processNoiseCov.at<float>(21) = 1e-1;
      kf_.processNoiseCov.at<float>(28) = 1e-4;
      kf_.processNoiseCov.at<float>(35) = 1e-4;
      
      cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar(20));
      ROS_INFO("MEASUREMENT ROWS: %d, MEASUREMENT COLS: %d", meas_.rows, meas_.cols);
      ROS_INFO("State ROWS: %d, State COLS: %d", state_.rows, state_.cols);
      ROS_INFO("MeasSize: %d", measSize_);
      
      ROS_INFO( "INIT COMPLETE" );

      tick_ = 0;

      for (int i = 0; i < 36; i= i+6)
      {
          ROS_INFO("[ %f   %f   %f   %f  %f  %f ] ",kf_.transitionMatrix.at<float>(i),kf_.transitionMatrix.at<float>(i+1),kf_.transitionMatrix.at<float>(i+2),kf_.transitionMatrix.at<float>(i+3),kf_.transitionMatrix.at<float>(i+4),kf_.transitionMatrix.at<float>(i+5));
      }

      ROS_INFO("\n");
        }

    ~trackerFilter() {
      cv::destroyWindow( "KF Result" );
    }

    void imageCallback( const sensor_msgs::ImageConstPtr& color_img );
    void trackerCallback( const sensor_msgs::RegionOfInterest& regionOfInterest );
    void KFPrediction();
    void updateKF();
    bool isTrackerInitialized();
};

