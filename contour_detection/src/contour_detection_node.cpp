#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <opencv2/core/ocl.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

static const std::string OPENCV_WINDOW = "Image window";

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
    /*
    std::string enc = cv_ptr->encoding;
    char enco[20];
    strcpy(enco, enc.c_str());
    */
    //ROS_INFO("Encoding: %s", enco);

    //  cv::COLOR_BGR2HSV 
    cv::Mat hsv;
    cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
    
    cv::Mat hsv_filtered;
    cv::inRange(hsv, cv::Scalar(50, 20, 100), cv::Scalar(70, 255, 255), hsv_filtered);

    //cv::Mat bgr;
    cv::Mat bgr = cv::Mat::zeros( hsv_filtered.size(), CV_32FC3 );
    cv::cvtColor(hsv_filtered, bgr, cv::COLOR_HSV2BGR_FULL);

    // convert the image to grayscale
    cv::Mat gray;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    // blur the image
    cv::blur( gray, gray, cv::Size(3,3) );
    // edge detection with our friend Kenny (Canny)
    int thresh = 100;
    cv::Mat canny_output;
    cv::Canny( gray, canny_output, thresh, thresh*2 );
    // now find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours( canny_output, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE );
    // For every found contour we now apply approximation to polygons 
    // with accuracy +-3 and stating that the curve must be closed.
    // After that we find a bounding rect for every polygon and save it to boundRect
    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    for( size_t i = 0; i < contours.size(); i++ )
    {
        cv::approxPolyDP( contours[i], contours_poly[i], 3, true );
        boundRect[i] = cv::boundingRect( contours_poly[i] );
    }
    // Create new Mat of unsigned 8-bit chars, filled with zeros.
    // It will contain all the drawings we are going to make (rects and circles).
    int type = CV_8UC3;
    cv::Mat drawing = cv::Mat::zeros( canny_output.size(), type );
    // For every contour: pick a random color, draw the contour, the bounding rectangle
    cv::RNG rng(12345);
    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        cv::drawContours( drawing, contours_poly, (int)i, color );
        cv::rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), color, 2 );
    }



    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, /* cv_ptr->image */ drawing);
    cv::waitKey(3);

    //ROS_INFO("I heard: %v", img->data);
}


int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "contour_detection_node");
  // get ros node handle
  ros::NodeHandle nh;

  // create a window the show things
  cv::namedWindow(OPENCV_WINDOW);
  cv::waitKey(3);
  
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




