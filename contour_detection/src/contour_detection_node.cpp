
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
#define RANGE_OF_AVERAGE 5


static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_RAW = "Image RAAAAAWW window";
static const std::string OPENCV_STRANGE = "Image filtered window";

int x_now_right;
int x_now_left;
bool x_last_flag_right = true;
bool x_last_flag_left = true;

int y_greenline = 30;

int x_greenlines_right[RANGE_OF_AVERAGE];
int x_greenlines_left[RANGE_OF_AVERAGE];

// right = true
// left = false
bool side = true;


// Are the two points set?
int is_set = 0;
// where the points updated?
int new_points = 0;

int find_right_point(cv::Mat blur){
  // find the highest x-point of the line
    int x_greenline = 140; // should be the highest x-point 
    // down most 45 lines 
    // find middle of the white stripe
    for( int y = 150; y < blur.rows - 15; y++ ) {
        for( int x = blur.cols - 5; x > 0; x-- ) {
            
            if(  80 < ( blur.at<uchar>(y,x)
              + blur.at<uchar>(y,x + 1)
              + blur.at<uchar>(y,x + 2)
              + blur.at<uchar>(y,x + 3)
            )
            ){
                //yaay
                x_greenline = x - 2;
                y_greenline = y;
                return x_greenline;
            }
        }
    }
    
}

int find_left_point(cv::Mat blur){
  // find the highest x-point of the line
    int x_greenline = 40; // should be the highest x-point 
    // down most 45 lines 
    // find middle of the white stripe
    for( int y = 150; y < blur.rows - 15; y++ ) {
        for( int x = 0; x < blur.cols - 5; x++ ) {
            
            if(  80 < ( blur.at<uchar>(y,x)
              + blur.at<uchar>(y,x + 1)
              + blur.at<uchar>(y,x + 2)
              + blur.at<uchar>(y,x + 3)
            )
            ){
                //yaay
                x_greenline = x + 2;
                y_greenline = y;
                return x_greenline;
            }
        }
    }
    
}

int average_right(int x_right_line){
  // initialization
    if(x_last_flag_right){
      for(int i = 0; i < RANGE_OF_AVERAGE; i++){
        x_greenlines_right[i] = x_right_line;
      }
      x_last_flag_right = false;
    }
    // refresh array 
    for(int i = 0; i < RANGE_OF_AVERAGE - 1; i++){
        x_greenlines_right[i + 1] = x_greenlines_right[i];
    }
    x_greenlines_right[0] = x_right_line;

    int x = 0;
    // average
    for(int i = 0; i < RANGE_OF_AVERAGE; i++){
        x += x_greenlines_right[i];
      }
    return x / RANGE_OF_AVERAGE;
}

int average_left(int x_left_line){
  // initialization
    if(x_last_flag_left){
      for(int i = 0; i < RANGE_OF_AVERAGE; i++){
        x_greenlines_left[i] = x_left_line;
      }
      x_last_flag_left = false;
    }
    // refresh array 
    for(int i = 0; i < RANGE_OF_AVERAGE - 1; i++){
        x_greenlines_left[i + 1] = x_greenlines_left[i];
    }
    x_greenlines_left[0] = x_left_line;

    int x = 0;
    // average
    for(int i = 0; i < RANGE_OF_AVERAGE; i++){
        x += x_greenlines_left[i];
      }
    return x / RANGE_OF_AVERAGE;
}


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
    int offset_x = 200;
    int offset_y = 200;
    cv::Rect roi;
    roi.x = offset_x;
    roi.y = offset_y;
    //roi.width = bird.size().width - (offset_x*2);
    roi.width = 240;
    //roi.height = bird.size().height - 150;
    roi.height = bird.size().height - 240;
    //Crop the original image to the defined ROI
    cv::Mat crop = bird(roi);
    
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
    cv::inRange(crop, cv::Scalar(62, 25, 148), cv::Scalar(85, 178, 255), hsv_filtered);
    
    // the image is now a grayscale

    // blur the image
    cv::Mat blur = cv::Mat::zeros(crop.size(), CV_8UC3);
    cv::GaussianBlur( hsv_filtered, blur, cv::Size( 15, 15 ), 0, 0 );
    
    
    int x_right_line = find_right_point(blur);

    int x_left_line = find_left_point(blur);
    
    x_now_right = average_right(x_right_line);
    x_now_left = average_left(x_left_line);

    ROS_INFO("left:\t%i \tright: \t%i",x_now_left,x_now_right);

    // mark the points as set 
    is_set = 1;
    // mark the points as updated
    new_points = 1;

    // visualize for debug
    cv::Mat bgr  = cv::Mat::zeros( hsv_filtered.size() * 3, CV_8UC3 );
    cv::cvtColor(blur, bgr,  cv::COLOR_GRAY2BGR, 3);
    cv::Scalar red = cv::Scalar( 0, 0, 255 );
    cv::Scalar blue = cv::Scalar( 255, 0, 0 );
    // display the point
    cv::rectangle( bgr, cv::Point(x_right_line,y_greenline), cv::Point(x_right_line,y_greenline) + cv::Point( 2,2 ), red, 2 );
    cv::rectangle( bgr, cv::Point(x_left_line,y_greenline), cv::Point(x_left_line,y_greenline) + cv::Point( 2,2 ), blue, 2 );
    
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
  
  // publish the two x points of the two lines
  ros::Publisher x1 = nh.advertise<std_msgs::Int32>("/line_recoqnition/x1", 1);
  ros::Publisher x2 = nh.advertise<std_msgs::Int32>("/line_recoqnition/x2", 1);


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
        std_msgs::Int32 x_2;
        x_1.data = x_now_right;
        x_2.data = x_now_left;
        // publish the points
        x1.publish( x_1 );
        x2.publish( x_2 );
      }
    }
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
