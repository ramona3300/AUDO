#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>








//  /kinect2/qhd/image_color/compressed

//  /dashboard/image_color_topic

// sensor_msgs/CompressedImage

//const image_msg::ConstPtr& img

void imageCallback(const sensor_msgs::Image::ConstPtr& img){
    
    ROS_INFO("I heard:vvv");
    //ROS_INFO("I heard: %v", img->data);
}


int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "contour_detection_node");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  std_msgs::Int16 motor, steering;

  /*
  // generate control message publisher
    ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
  ros::Publisher chatter = nh.advertise<std_msgs::String>("chatter",10);
  */
  //ros::Subscriber sub = nh.subscribe("/kinect2/qhd/image_color/compressed", 10, imageCallback);

  //ros::Subscriber sub = nh.subscribe<image_msg>("kinect2/qhd/image_color", 10, imageCallback);
  //ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("kinect2/qhd/image_color", 10, imageCallback);
  ros::Subscriber sub = nh.subscribe("kinect2/qhd/image_color", 10, imageCallback);

  ROS_INFO("Contour Detection Start");

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(0.5);
  while (ros::ok())
  {
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}




