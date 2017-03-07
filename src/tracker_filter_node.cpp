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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tracker_filter");

  ROS_INFO( "ROS INIT COMPLETE");
  trackerFilter tracker;

  ros::Time begin = ros::Time::now();
  ros::Rate loop_rate(10);
  cv::namedWindow("KF Result");
  ros::spin();

  /*
  while ( ros::ok() ) {
    ros::spinOnce();
    //tracker.
    loop_rate.sleep();
  }
  */
}
