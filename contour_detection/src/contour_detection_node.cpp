
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
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
#include <math.h>


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_RAW = "Image RAAAAAWW window";
static const std::string OPENCV_STRANGE = "Image filtered window";

int x_now;

// Are the two points set?
int is_set = 0;
// where the points updated?
int new_points = 0;


void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    
    ROS_INFO("Received an Image!");

    // save image to a cv::Mat
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
    
    // #####################################################################################################
    // #####  Transform perpective to birdeye view                                            ##############
    // #####################################################################################################

    // Input Quadilateral or Image plane coordinates
    cv::Point2f inputQuad[4]; 
    // The 4 points that select quadilateral on the input , from top-left in clockwise order
    // These four pts are the sides of the rect box used as input 
    inputQuad[0] = cv::Point2f( 1,150 ); //topleft
    inputQuad[1] = cv::Point2f( 639,150 ); //topright
    inputQuad[2] = cv::Point2f( 639,479 ); //bottomright
    inputQuad[3] = cv::Point2f( 1,479  ); //bottomleft
    // Output Quadilateral or World plane coordinates
    cv::Point2f outputQuad[4];
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = cv::Point2f( 1,1 ); //topleft mapped
    outputQuad[1] = cv::Point2f( 639,1 ); //topright mapped
    outputQuad[2] = cv::Point2f( 380,439 ); //bottomright mapped
    outputQuad[3] = cv::Point2f( 260,439  ); //bottomleft mapped

    // Lambda Matrix
    cv::Mat lambda;
    // Get the Perspective Transform Matrix i.e. lambda 
    lambda = cv::getPerspectiveTransform( inputQuad, outputQuad );
    cv::Mat bird  = cv::Mat::zeros( (cv_ptr->image).size(), (cv_ptr->image).type() );
    cv::warpPerspective(cv_ptr->image,bird,lambda,bird.size() );
    
    // #####################################################################################################
    // #####  Crop the birdeye view to a rectangle                                            ##############
    // #####################################################################################################
    
    // Set Region of Interest
    int offset_x = 260;
    int offset_y = 100;
    cv::Rect roi;
    roi.x = offset_x;
    roi.y = offset_y;
    //roi.width = bird.size().width - (offset_x*2);
    roi.width = 120;
    //roi.height = bird.size().height - 150;
    roi.height = bird.size().height - 150;
    //Crop the original image to the defined ROI
    cv::Mat crop = bird(roi);
    

    // #####################################################################################################
    // #####  Make the image brighter for better color recoqnition                            ##############
    // #####################################################################################################
    //cv::Mat brighton = cv::Mat::zeros(crop.size(), CV_8UC3);
    double alpha = 1.0; /*< Simple contrast control [1.0-3.0]*/
    int beta = 0;       /*< Simple brightness control [0-100]*/
    /*
    for( int y = 0; y < crop.rows; y++ ) {
        for( int x = 0; x < crop.cols; x++ ) {
            for( int c = 0; c < crop.channels(); c++ ) {
                crop.at<cv::Vec3b>(y,x)[c] =
                  cv::saturate_cast<uchar>( alpha*crop.at<cv::Vec3b>(y,x)[c] + beta );
            }
        }
    }
    */
    cv::imshow(OPENCV_STRANGE, crop);
    cv::waitKey(3);
    // #####################################################################################################
    // #####  Filter out everything that isn't green                                          ##############
    // #####################################################################################################
    
    // convert image from BGR to HSV
    //cv::Mat hsv1  = cv::Mat::zeros( crop.size(), CV_8UC3 );
    cv::cvtColor(crop, crop, cv::COLOR_BGR2HSV, 3);
    // only keep green
    cv::Mat hsv_filtered   = cv::Mat::zeros( crop.size(), CV_8UC3 );
    //cv::inRange(hsv1, cv::Scalar(50, 20, 100), cv::Scalar(70, 255, 255), hsv_filtered);
    cv::inRange(crop, cv::Scalar(62, 20, 100), cv::Scalar(78, 255, 255), hsv_filtered);
    
    // the image is now a grayscale

    // blur the image
    cv::Mat blur = cv::Mat::zeros(crop.size(), CV_8UC3);
    cv::GaussianBlur( hsv_filtered, blur, cv::Size( 15, 15 ), 0, 0 );
    
    //cv::imshow(OPENCV_RAW, blur);
    //cv::waitKey(3);

    // find the lowest x-point of the line
    int x_ding = 80; // should be the lowest x-point 
    int y_ding = 30;
    // down most 45 lines 
    // find middle of the white stripe
    for( int y = 280; y < blur.rows - 15; y++ ) {
        for( int x = 0; x < blur.cols - 5; x++ ) {
            
            if(  100 < ( blur.at<uchar>(x,y)
              + blur.at<uchar>(y,x + 1 )
              + blur.at<uchar>(y,x + 2)
              + blur.at<uchar>(y,x + 3)
              + blur.at<uchar>(y,x + 4)
            )
            ){
                //yaay
                x_ding = x + 1;
                y_ding = y;
                break;
            }
        }
    }
    x_now = x_ding;
    ROS_INFO("Lowest X of line:\t%i",x_ding);

    // mark the points as set 
    is_set = 1;
    // mark the points as updated
    new_points = 1;

    // visualize for debug
    cv::Mat bgr  = cv::Mat::zeros( hsv_filtered.size() * 3, CV_8UC3 );
    cv::cvtColor(blur, bgr,  cv::COLOR_GRAY2BGR, 3);
    cv::Scalar red = cv::Scalar( 0, 0, 255 );
    cv::Scalar blue = cv::Scalar( 255, 0, 0 );
    // display the relevant line
    //line(bgr, cv::Point(vec[2],vec[3]), cv::Point(vec[0],vec[1]), red, 3, 8, 0);
    //line(bgr, lower, upper, blue, 2, 8, 0);
    cv::rectangle( bgr, cv::Point(x_ding,y_ding), cv::Point(x_ding,y_ding) + cv::Point( 2,2 ), red, 2 );
    // show image
    cv::imshow(OPENCV_WINDOW, bgr);
    cv::waitKey(3);
    
  
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
  cv::namedWindow(OPENCV_STRANGE);
  cv::waitKey(3);
  //cv::namedWindow(OPENCV_RAW);
  //cv::waitKey(3);
  
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);
  //image_transport::Subscriber sub = it.subscribe("kinect2/qhd/image_color", 1, imageCallback);
  
  // publish the line as four points
  ros::Publisher x1 = nh.advertise<std_msgs::Int32>("/line_recoqnition/x1", 1);
  


  ROS_INFO("Contour Detection Start");

  
  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    if( is_set ){
      // only try to publish points if they where initialized
      if( new_points ){
        // only publish points if they where updated
        new_points = 0;
        std_msgs::Int32 x_1;
        x_1.data = x_now;
        // publish the points
        x1.publish( x_1 );
      }
    }
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
