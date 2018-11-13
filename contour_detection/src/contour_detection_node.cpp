#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>





static const std::string OPENCV_WINDOW = "Image window";

//  /kinect2/qhd/image_color/compressed

//  /dashboard/image_color_topic

// sensor_msgs/CompressedImage

//const image_msg::ConstPtr& img

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    
    ROS_INFO("I heard:vvv");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);

    //ROS_INFO("I heard: %v", img->data);
}


int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "contour_detection_node");
  // get ros node handle
  ros::NodeHandle nh;

  //cv::namedWindow(OPENCV_WINDOW);
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("kinect2/qhd/image_color", 1, imageCallback);
  //image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);
  
  
  

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




