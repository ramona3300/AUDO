
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

cv::Point upper;
cv::Point lower;

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
    inputQuad[0] = cv::Point2f( 1,400 ); //topleft
    inputQuad[1] = cv::Point2f( 1279,400 ); //topright
    inputQuad[2] = cv::Point2f( 1279,719 ); //bottomright
    inputQuad[3] = cv::Point2f( 1,719  ); //bottomleft
    // Output Quadilateral or World plane coordinates
    cv::Point2f outputQuad[4];
    // The 4 points where the mapping is to be done , from top-left in clockwise order
    outputQuad[0] = cv::Point2f( 1,1 ); //topleft mapped
    outputQuad[1] = cv::Point2f( 1279,1 ); //topright mapped
    outputQuad[2] = cv::Point2f( 800,719 ); //bottomright mapped
    outputQuad[3] = cv::Point2f( 300,719  ); //bottomleft mapped

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
    int offset_x = 300;
    int offset_y = 0;
    cv::Rect roi;
    roi.x = offset_x;
    roi.y = offset_y;
    roi.width = bird.size().width - (offset_x*2);
    roi.height = bird.size().height - 150;
    //Crop the original image to the defined ROI
    cv::Mat crop = bird(roi);
    

    // #####################################################################################################
    // #####  Make the image brighter for better color recoqnition                            ##############
    // #####################################################################################################
    //cv::Mat brighton = cv::Mat::zeros(crop.size(), CV_8UC3);
    double alpha = 1.0; /*< Simple contrast control [1.0-3.0]*/
    int beta = 10;       /*< Simple brightness control [0-100]*/
    
    for( int y = 0; y < crop.rows; y++ ) {
        for( int x = 0; x < crop.cols; x++ ) {
            for( int c = 0; c < crop.channels(); c++ ) {
                crop.at<cv::Vec3b>(y,x)[c] =
                  cv::saturate_cast<uchar>( alpha*crop.at<cv::Vec3b>(y,x)[c] + beta );
            }
        }
    }
    
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
    cv::inRange(crop, cv::Scalar(40, 30, 100), cv::Scalar(70, 255, 255), hsv_filtered);
    
    // the image is now a grayscale

    cv::Mat blur = cv::Mat::zeros(crop.size(), CV_8UC3);
    cv::GaussianBlur( hsv_filtered, blur, cv::Size( 15, 15 ), 0, 0 );
    
    cv::imshow(OPENCV_RAW, blur);
    cv::waitKey(3);

    // #####################################################################################################
    // #####  Convert the image from grayscale to binary (only black or white)                ##############
    // #####################################################################################################
    
    cv::Mat bin;
    //cv::threshold( hsv_filtered, bin, threshold_value, max_BINARY_value,threshold_type );
    cv::threshold( blur, bin, 245, 255,CV_THRESH_BINARY );

    // #####################################################################################################
    // #####  Detect straight lines with the probalistic Hough transformation                 ##############
    // #####################################################################################################    

    std::vector<cv::Vec4i> lines;
    //threshold: The minimum number of intersections to “detect” a line
    //minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    //maxLineGap: The maximum gap between two points to be considered in the same line.
    //cv::HoughLinesP(bin, lines, 1, CV_PI/180, threshold, minLinLength, maxLineGap );
    cv::HoughLinesP(bin, lines, 1, CV_PI/180, 100, 100, 50 );

    if(lines.size() == 0){
      return; // there is nothing more to do
    }

    // calculate some more information about the found lines
    ROS_INFO("lines:\t%i",(int)(lines.size()));
    int lengths[lines.size()];
    double alphas[lines.size()];
    for( int i = 0; i < lines.size(); i++ ) {
      cv::Vec4i vec = lines[i];
      lengths[i] = sqrt( pow( abs( vec[2] - vec[0] ), 2 ) + pow( abs( vec[3] - vec[1] ), 2 ) );
      alphas[i] = asin( (double) (abs( vec[3] - vec[1] )) / (double)(lengths[i]) ) * 180 / M_PI;
      //ROS_INFO("line[%i]: x1 = %i, y1 = %i, x2 = %i; y2 = %i, length = %i, alpha = %f",i,vec[0],vec[1],vec[2],vec[3],lengths[i],alphas[i]);
    }
    // To Do: write code for the left side
    // CASE: driving on the right side
    // search for the line with smallest x and biggest y start point
    int x_min = 300;
    int y_max = 0;
    for( int i = 0; i < lines.size(); i++ ) {
      cv::Vec4i vec = lines[i];
      if( x_min > vec[2] && vec[3] > 300){
        x_min = vec[2];
        if( y_max < vec[3]){
          y_max = vec[3];
        }
      }
    }
    // find the relevant line
    int start = 0;
    for( int i = 0; i < lines.size(); i++ ) {
      cv::Vec4i vec = lines[i];
      if( x_min + 10 >= vec[2] 
          && vec[2] >= x_min - 10 
          && y_max + 10 >= vec[3] 
          && vec[3] >= y_max - 10 ){
        start = i;
        break;
      }
    }
    // small hack if the relevant line was not found correctly - works surprisingly well
    if(start == 0){
      start = 1;
    }
    ROS_INFO("start = %i",start);
    
    // AUDO is in the middle of the right track if the start point of the line is at x = 260px
    // height = 390
    // width = 360
    // blindspot length = 35cm, meassured from alu case
    // 10cm_real = 4,3cm_screen
    // pixel/cm_real = 15,97
    // Extend line
    cv::Vec4i vec = lines[start];
    double m = ( (double)(vec[2]) - (double)(vec[0]) ) / ( (double)(vec[3]) - (double)(vec[1]) );
    double b = vec[2] - m * vec[3];
    upper = cv::Point((int)( vec[0] - m * vec[1] ),0);
    lower = cv::Point((int) (m * bin.size().width + b), (int) bin.size().height);
    
    // mark the points as set 
    is_set = 1;
    // mark the points as updated
    new_points = 1;


    ROS_INFO("m = %f, Upper = (%i,%i), Lower = (%i,%i)", m, upper.x, upper.y, lower.x, lower.y);
    ROS_INFO("line[%i]: x1 = %i, y1 = %i, x2 = %i; y2 = %i, length = %i, alpha = %f",start,vec[0],vec[1],vec[2],vec[3],lengths[start],alphas[start]);

    // visualize for debug
    cv::Mat bgr  = cv::Mat::zeros( hsv_filtered.size() * 3, CV_8UC3 );
    cv::cvtColor(hsv_filtered, bgr,  cv::COLOR_GRAY2BGR, 3);
    cv::Scalar red = cv::Scalar( 0, 0, 255 );
    cv::Scalar blue = cv::Scalar( 255, 0, 0 );
    // display the relevant line
    line(bgr, cv::Point(vec[2],vec[3]), cv::Point(vec[0],vec[1]), red, 3, 8, 0);
    line(bgr, lower, upper, blue, 2, 8, 0);
    // show image
    cv::imshow(OPENCV_WINDOW, bgr);
    cv::waitKey(3);
    

    // Color filter attempts for other Webcam
    //cv::inRange(hsv, cv::Scalar(100, 20, 100), cv::Scalar(115, 255, 255), hsv_filtered);
    // H 107
    // S 255
    // V 224
    //cv::GaussianBlur( hsv_filtered, lowPassed, Size( 5, 5 ), 2, 2 );
    // cv::Mat lowPassed;
    //cv::bilateralFilter(hsv_filtered, lowPassed, 5, 120, 120);
  
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
  cv::namedWindow(OPENCV_RAW);
  cv::waitKey(3);
  
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("kinect2/qhd/image_color", 1, imageCallback);
  
  // publish the line as four points
  ros::Publisher x1 = nh.advertise<std_msgs::Int32>("/line_recoqnition/x1", 1);
  ros::Publisher y1 = nh.advertise<std_msgs::Int32>("/line_recoqnition/y1", 1);
  ros::Publisher x2 = nh.advertise<std_msgs::Int32>("/line_recoqnition/x2", 1);
  ros::Publisher y2 = nh.advertise<std_msgs::Int32>("/line_recoqnition/y2", 1);
  


  ROS_INFO("Contour Detection Start");

  
  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(0.5);
  while (ros::ok())
  {

    if( is_set ){
      // only try to publish points if they where initialized
      if( new_points ){
        // only publish points if they where updated
        new_points = 0;

        std_msgs::Int32 x_1;
        std_msgs::Int32 y_1;
        std_msgs::Int32 x_2;
        std_msgs::Int32 y_2;

        x_1.data = lower.x;
        y_1.data = lower.y;
        x_2.data = upper.x;
        y_2.data = upper.y;

        // publish the points
        x1.publish( x_1 );
        y1.publish( y_1 );
        x2.publish( x_2 );
        y2.publish( y_2 );
      }
    }
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
