
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
#define NO_OBSTACLE 9999
// State of Obstacle Search 
#define OD_FIND_RIGHT_LINE 1
#define OD_FIND_LEFT_LINE 2
#define OD_FIND_VOID 3



static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_RAW = "Image RAAAAAWW window";
static const std::string OPENCV_STRANGE = "Image filtered window";

int x_now_right;
int x_now_left;

int x_right_line_2;
int x_left_line_2;
int x_right_line_3;
int x_left_line_3;

int x_obstacle[2];


bool x_last_flag_right = true;
bool x_last_flag_left = true;

int y_greenline = 30;

int y_coords_left[4];
int y_coords_right[4];

int x_greenlines_right[RANGE_OF_AVERAGE];
int x_greenlines_left[RANGE_OF_AVERAGE];

bool side = true;

// Are the two points set?
int is_set = 0;
// where the points updated?
int new_points = 0;

// blur.rows = blur.cols =240
// TODO Punkte weiter hoch verschieben, um schwarze Dreiecke zu vermeiden
int find_right_point(cv::Mat blur, int height, int index){
  // find the highest x-point of the line
    int x_greenline = 140; // should be the highest x-point 
    // down most 45 lines 
    // find middle of the white stripe
    for( int y = height; y < blur.rows - 15; y++ ) {
        for( int x = blur.cols - 5; x > 0; x-- ) {
            //4 folgende Pixel, die zusammen heller als 80 sind
            if(  80 < ( blur.at<uchar>(y,x)
              + blur.at<uchar>(y,x + 1)
              + blur.at<uchar>(y,x + 2)
              + blur.at<uchar>(y,x + 3)
            )
            ){
                //yaay
                x_greenline = x - 5;
                y_coords_right[index] = y;
                return x_greenline;
            }
        }
    }
    
}

// blur.rows = blur.cols =240
int find_right_point_alternative(cv::Mat blur, int height, int index){
  // find the highest x-point of the line
    int x_greenline = 140; // should be the highest x-point 
    // down most 45 lines 
    // find middle of the white stripe
    for( int y = blur.rows - 60; y > height; y-- ) {
        for( int x = blur.cols - 5; x > 80; x-- ) {
            //4 folgende Pixel, die zusammen heller als 80 sind
            if(  80 < ( blur.at<uchar>(y,x)
              + blur.at<uchar>(y,x + 1)
              + blur.at<uchar>(y,x + 2)
              + blur.at<uchar>(y,x + 3)
            )
            ){
                //yaay
                x_greenline = x - 5;
                y_coords_right[index] = y;
                return x_greenline;
            }
        }
    }
    return 140;
    
}

// blur.rows = blur.cols =240
int find_left_point_alternative(cv::Mat blur, int height, int index){
  // find the highest x-point of the line
    int x_greenline = 40; // should be the highest x-point 
    // down most 45 lines 
    // find middle of the white stripe
    for( int y = blur.rows - 60; y > height; y--) {
        for( int x = 5; x < blur.cols - 80; x++ ) {
            
            if(  80 < ( blur.at<uchar>(y,x)
              + blur.at<uchar>(y,x + 1)
              + blur.at<uchar>(y,x + 2)
              + blur.at<uchar>(y,x + 3)
            )
            ){
                //yaay
                x_greenline = x + 5;
                y_coords_left[index] = y;
                return x_greenline;
            }
        }
    }
    return 40;
    
}

// blur.rows = blur.cols =240
int find_left_point(cv::Mat blur, int height, int index){
  // find the highest x-point of the line
    int x_greenline = 40; // should be the highest x-point 
    // down most 45 lines 
    // find middle of the white stripe
    for( int y = height; y < blur.rows - 15; y++ ) {
        for( int x = 0; x < blur.cols - 5; x++ ) {
            
            if(  80 < ( blur.at<uchar>(y,x)
              + blur.at<uchar>(y,x + 1)
              + blur.at<uchar>(y,x + 2)
              + blur.at<uchar>(y,x + 3)
            )
            ){
                //yaay
                x_greenline = x + 5;
                y_coords_left[index] = y;
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

void obstacle_detection(cv::Mat blur, int height, int index, int* obstacle){
  // find the highest x-point of the line
    int state = OD_FIND_RIGHT_LINE;
    int x_right_od = NO_OBSTACLE;
    int x_left_od = NO_OBSTACLE;
    // find white stripe
    for( int y = 130; y > height; y-- ) {
        state = OD_FIND_RIGHT_LINE;
        for( int x = blur.cols - 20; x > 20; x-- ) {
          switch (state)
          {
            case OD_FIND_RIGHT_LINE:
              //4 folgende Pixel, die zusammen heller als 80 sind
              if(  80 < ( blur.at<uchar>(y,x)
                + blur.at<uchar>(y,x + 1)
                + blur.at<uchar>(y,x + 2)
                + blur.at<uchar>(y,x + 3)
              )){
                x_right_od = x - 5;
                y_coords_right[index] = y;
                state = OD_FIND_VOID;
            }
              break;
            case OD_FIND_VOID:
              //5 folgende Pixel, die zusammen dunkler als 20 sind
              if(  20 > ( blur.at<uchar>(y,x)
                + blur.at<uchar>(y,x + 1)
                + blur.at<uchar>(y,x + 2)
                + blur.at<uchar>(y,x + 3)
                + blur.at<uchar>(y,x + 4)
              )){
                state = OD_FIND_LEFT_LINE;
              }
              break;
            case OD_FIND_LEFT_LINE:
              //4 folgende Pixel, die zusammen heller als 80 sind
              if(  80 < ( blur.at<uchar>(y,x)
                + blur.at<uchar>(y,x + 1)
                + blur.at<uchar>(y,x + 2)
                + blur.at<uchar>(y,x + 3)
              )){
                x_left_od = x - 5;
                obstacle[0] = x_right_od;
                obstacle[1] = x_left_od;
                return;
              }
              break;
          }
        }
    }
    obstacle[0] = NO_OBSTACLE; // default value, no obstacle detected
    obstacle[1] = NO_OBSTACLE;
    return;
}

// receive and process images
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    //ROS_INFO("Received an Image!");
    // #####################################################################################################
    // ##### Save the image to a cv::Mat for further processing                               ##############
    // #####################################################################################################
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
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
    // debug view
    cv::imshow(OPENCV_STRANGE, crop);
    cv::waitKey(3);
    // #####################################################################################################
    // #####  Filter out everything that isn't green                                          ##############
    // #####################################################################################################
    // convert image from BGR to HSV
    cv::cvtColor(crop, crop, cv::COLOR_BGR2HSV, 3);
    // only keep green
    cv::Mat hsv_filtered   = cv::Mat::zeros( crop.size(), CV_8UC3 );
    cv::inRange(crop, cv::Scalar(62, 25, 148), cv::Scalar(85, 178, 255), hsv_filtered);
    // only keep orange
    cv::Mat hsv_filtered_or   = cv::Mat::zeros( crop.size(), CV_8UC3 );
    cv::inRange(crop, cv::Scalar(2, 150, 180), cv::Scalar(15, 255, 255), hsv_filtered_or);
    
    // the image is now a grayscale
    // #####################################################################################################
    // #####  Blur the images                                                                 ##############
    // #####################################################################################################
    cv::Mat blur = cv::Mat::zeros(crop.size(), CV_8UC3);
    cv::GaussianBlur( hsv_filtered, blur, cv::Size( 15, 15 ), 0, 0 );
    cv::GaussianBlur( hsv_filtered_or, hsv_filtered_or, cv::Size( 15, 15 ), 0, 0 );
    // #####################################################################################################
    // #####  Find a point at some distance on both lines                                     ##############
    // #####################################################################################################
    int x_right_line = find_right_point_alternative(blur, 150, 0);
    int x_left_line = find_left_point_alternative(blur, 150, 0);
    // average with the last couple of points to mitigate hickups
    // maybe weigthed average instead
    x_now_right = average_right(x_right_line);
    x_now_left = average_left(x_left_line); 
    // x_now_left should be 80 but for the controller to function properly 
    // we add 60 to get to 140, because the controller is tuned for 140

    // find two more points to differentiate between curves and straights 
    x_right_line_2 = find_right_point(blur, 115, 1);
    x_left_line_2 = find_left_point(blur, 115, 1);
    x_right_line_3 = find_right_point(blur, 50, 2);
    x_left_line_3 = find_left_point(blur, 50, 2);

    // mark the points as set 
    is_set = 1;
    // mark the points as updated
    new_points = 1;

    // #####################################################################################################
    // #####  Obstacle detection                                                             ##############
    // #####################################################################################################
    
    obstacle_detection(hsv_filtered_or, 90, 3, x_obstacle);

    // #####################################################################################################
    // #####  Visualize for debug                                                             ##############
    // #####################################################################################################
    ROS_INFO("left:\t%i \tright: \t%i",x_now_left,x_now_right);
    cv::Mat bgr  = cv::Mat::zeros( hsv_filtered.size() * 3, CV_8UC3 );
    cv::cvtColor(blur, bgr,  cv::COLOR_GRAY2BGR, 3);

    cv::Mat bgr_or  = cv::Mat::zeros( hsv_filtered_or.size() * 3, CV_8UC3 );
    cv::cvtColor(hsv_filtered_or, bgr_or,  cv::COLOR_GRAY2BGR, 3);

    cv::Scalar red = cv::Scalar( 0, 0, 255 );
    cv::Scalar green = cv::Scalar( 0, 255, 0 );
    cv::Scalar orange = cv::Scalar( 0, 165, 254 );
    // display the points
    cv::rectangle( bgr, cv::Point(x_right_line,y_coords_right[0]), cv::Point(x_right_line,y_coords_right[0]) + cv::Point( 2,2 ), green, 2 );
    cv::rectangle( bgr, cv::Point(x_left_line,y_coords_left[0]), cv::Point(x_left_line,y_coords_left[0]) + cv::Point( 2,2 ), red, 2 );
    cv::rectangle( bgr, cv::Point(x_right_line_2,y_coords_right[1]), cv::Point(x_right_line_2,y_coords_right[1]) + cv::Point( 2,2 ), green, 2 );
    cv::rectangle( bgr, cv::Point(x_left_line_2,y_coords_left[1]), cv::Point(x_left_line_2,y_coords_left[1]) + cv::Point( 2,2 ), red, 2 );
    cv::rectangle( bgr, cv::Point(x_right_line_3,y_coords_right[2]), cv::Point(x_right_line_3,y_coords_right[2]) + cv::Point( 2,2 ), green, 2 );
    cv::rectangle( bgr, cv::Point(x_left_line_3,y_coords_left[2]), cv::Point(x_left_line_3,y_coords_left[2]) + cv::Point( 2,2 ), red, 2 );
    cv::rectangle( bgr, cv::Point(x_obstacle[0],y_coords_right[3]), cv::Point(x_obstacle[0],y_coords_right[3]) + cv::Point( 2,2 ), orange, 2 );
    cv::rectangle( bgr, cv::Point(x_obstacle[1],y_coords_right[3]), cv::Point(x_obstacle[1],y_coords_right[3]) + cv::Point( 2,2 ), orange, 2 );
    cv::rectangle( bgr_or, cv::Point(x_obstacle[0],y_coords_right[3]), cv::Point(x_obstacle[0],y_coords_right[3]) + cv::Point( 2,2 ), orange, 2 );
    cv::rectangle( bgr_or, cv::Point(x_obstacle[1],y_coords_right[3]), cv::Point(x_obstacle[1],y_coords_right[3]) + cv::Point( 2,2 ), orange, 2 );
    
    // show images
    cv::imshow(OPENCV_WINDOW, bgr);
    cv::waitKey(3);
    cv::imshow(OPENCV_RAW, bgr_or);
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
  cv::namedWindow(OPENCV_RAW);
  cv::waitKey(3);
  
  // subscribe to receive video feed
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);
  // publish the two x points of the two lines
  ros::Publisher right_pub = nh.advertise<std_msgs::Int32>("/line_recoqnition/right", 1);
  ros::Publisher left_pub = nh.advertise<std_msgs::Int32>("/line_recoqnition/left", 1);
  ros::Publisher right_pub_2 = nh.advertise<std_msgs::Int32>("/line_recoqnition/right_2", 1);
  ros::Publisher left_pub_2 = nh.advertise<std_msgs::Int32>("/line_recoqnition/left_2", 1);
  ros::Publisher right_pub_3 = nh.advertise<std_msgs::Int32>("/line_recoqnition/right_3", 1);
  ros::Publisher left_pub_3 = nh.advertise<std_msgs::Int32>("/line_recoqnition/left_3", 1);
  ros::Publisher obstacle_pub_right = nh.advertise<std_msgs::Int32>("/line_recoqnition/obstacle/right", 1);
  ros::Publisher obstacle_pub_left = nh.advertise<std_msgs::Int32>("/line_recoqnition/obstacle/left", 1);
  // Loop starts here:
  ROS_INFO("Contour Detection Start");
  // loop rate value is set in Hz
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    if( is_set ){
      // only try to publish points if they where initialized
      if( new_points ){
        // only publish points if they where updated
        new_points = 0;
        std_msgs::Int32 x_right;
        std_msgs::Int32 x_left;
        std_msgs::Int32 x_right_2;
        std_msgs::Int32 x_left_2;
        std_msgs::Int32 x_right_3;
        std_msgs::Int32 x_left_3;
        std_msgs::Int32 obstacle_right;
        std_msgs::Int32 obstacle_left;
        
        x_right.data = x_now_right;
        x_left.data = x_now_left;
        x_right_2.data = x_right_line_2;
        x_left_2.data = x_left_line_2;
        x_right_3.data = x_right_line_3;
        x_left_3.data = x_left_line_3;
        obstacle_right.data = x_obstacle[0];
        obstacle_left.data = x_obstacle[1];
        
        // publish the points
        right_pub.publish( x_right );
        left_pub.publish( x_left );
        right_pub_2.publish( x_right_2 );
        left_pub_2.publish( x_left_2 );
        right_pub_3.publish( x_right_3 );
        left_pub_3.publish( x_left_3 );
        obstacle_pub_right.publish( obstacle_right );
        obstacle_pub_left.publish( obstacle_left );
        
      }
    }
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
