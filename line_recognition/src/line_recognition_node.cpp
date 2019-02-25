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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <math.h>
// range of the averages over main left and right points
#define RANGE_OF_AVERAGE 5
// indicate that no obstacle was found
#define NO_OBSTACLE 9999
// State of Obstacle Search 
#define OD_FIND_RIGHT_LINE 1
#define OD_FIND_LEFT_LINE 2
#define OD_FIND_VOID 3
// used for image_regional_value_check to indicate to check for less ore more than something
#define MORE 1
#define LESS 0
// defining the names of the debug windows
static const std::string OPENCV_WINDOW = "Line Recgnition";
static const std::string OPENCV_RAW = "Obstacle Detection";
//static const std::string OPENCV_STRANGE = "Webcam: Bird-Eye with Region of Interest";
static const std::string OPENCV_STRANGE = "Webcam Image";
static const std::string OPENCV_STRANGE2 = "Webcam: Bird-Eye with Region of Interest";
static const std::string OPENCV_KINECT = "Kinect Image";
static const std::string OPENCV_KINECT2 = "Kinect: Bird-Eye with Region of Interest";
// main points
int x_now_right;
int x_now_left;
// secondary points
int x_right_line_2;
int x_left_line_2;
int x_right_line_3;
int x_left_line_3;
// position of the obstacle
int x_obstacle[2];
// store old points to later average them out
int x_greenlines_right[RANGE_OF_AVERAGE];
int x_greenlines_left[RANGE_OF_AVERAGE];
// indicate wether or not the above array were initialized
int x_right_init = 1;
int x_left_init = 1;
// store y axis points for debu diplay
int y_coords_left[5];
int y_coords_right[5];
// Are the points set?
int is_set = 0;
// where the points updated?
int new_points = 0;

/*
  image_regional_value_check
  Checks if the sum of the values of an image
  in a specified region is greater or lesser than
  a specific value
  cv::Mat image       the grayscaled image
  int x               Point on x axis
  int y               Point on y axis
  int range           Range to be checked
  int value           Value to be checked against
  int more_or_less    Check for greater or lesser than the specified value
*/
int image_regional_value_check(cv::Mat image, int x, int y, int range, int value, int more_or_less)
{
  int sum = 0;
  for(int i = 0; i < range; i++)
  {
    sum += image.at<uchar>(y,x + i);
  }
  if(more_or_less == MORE)
  { 
    return value < sum;
  }
  if(more_or_less == LESS)
  { 
    return value > sum;
  }  
}

/*
  find_secondary_right_point
  Searches the image for a point on the right line
  at a specific distance (with some margin of error) to the car.
  The points found with this function will only be used to differentiate curves from straights.
  Therefor it searches horizontally for the first white area from a specific start point.
  The region is schematically shown below.
  __________________________
  |                        |
  |               <== Point|
  |                    ||  |
  |                    \/  |
  |                        |
  |                        |
  |                        |
  |                        |
  |                        |
  |________________________|
*/
int find_secondary_right_point(cv::Mat image, int height, int index)
{
  int x_greenline = 140;// sensible default 
  for( int y = height; y < image.rows - 15; y++ ) 
  {
    for( int x = image.cols - 5; x > 0; x-- ) 
    {
      if(image_regional_value_check(image, x, y, 4, 80, MORE))
      {
        x_greenline = x - 5;
        y_coords_right[index] = y;
        return x_greenline;
      }
    }
  }
}

/*
  find_main_right_point
  Searches the image for a point on the right line
  at a specific distance (with some margin of error) to the car.
  This Point will be the nearest point found and will be used for controlling.
  Therefor it searches horizontally for the first white area in a specified region.
  The region is schematically shown below.
  __________________________
  |                        |
  |                        |
  |                        |
  |                        |
  |                        |
  |                        |
  |                ________|
  |                | This  |
  |                | Area  |
  |________________|_______|
*/
int find_main_right_point(cv::Mat image, int height, int index)
{
  int x_greenline = 140; // sensible default 
  for( int y = image.rows - 60; y > height; y-- ) 
  {
    for( int x = image.cols - 5; x > 80; x-- ) 
    {
      if(image_regional_value_check(image, x, y, 4, 80, MORE))
      {
        x_greenline = x - 5;
        y_coords_right[index] = y;
        return x_greenline;
        }
      }
    }
  return x_greenline;
}
/*
  find_main_left_point
  Searches the image for a point on the left line
  at a specific distance (with some margin of error) to the car.
  This Point will be the nearest point found and will be used for controlling.
  Therefor it searches horizontally for the first white area in a specified region.
  The region is schematically shown below.
  __________________________
  |                        |
  |                        |
  |                        |
  |                        |
  |                        |
  |                        |
  |________                |
  | This   |               |
  | Area   |               |
  |________|_______________|
*/
int find_main_left_point(cv::Mat image, int height, int index)
{
  int x_greenline = 40; // sensible default 
  for( int y = image.rows - 60; y > height; y--) 
  {
    for( int x = 5; x < image.cols - 80; x++ ) 
    {     
      if(image_regional_value_check(image, x, y, 4, 80, MORE))
      {
         x_greenline = x + 5;
         y_coords_left[index] = y;
         return x_greenline;
      }
    }
  }
  return x_greenline;
}

/*
  find_secondary_left_point
  Searches the image for a point on the left line
  at a specific distance (with some margin of error) to the car.
  The points found with this function will only be used to differentiate curves from straights.
  Therefor it searches horizontally for the first white area from a specific start point.
  The region is schematically shown below.
  __________________________
  |                        |
  |Point ==>               |
  |  ||                    |
  |  \/                    |
  |                        |
  |                        |
  |                        |
  |                        |
  |                        |
  |________________________|
*/
int find_secondary_left_point(cv::Mat image, int height, int index)
{
  int x_greenline = 40; // sensible default 
  for( int y = height; y < image.rows - 15; y++ ) 
  {
    for( int x = 0; x < image.cols - 5; x++ ) 
    {
      if(image_regional_value_check(image, x, y, 4, 80, MORE))
      {
         x_greenline = x + 5;
         y_coords_left[index] = y;
         return x_greenline;
      }
    }
  }
  return x_greenline;
}

/*
  average
  Averages the last couple of integers, given an array of the last integers
  and a range to average over.
  It also can initialize the array if needed.

  int newest        newest value 
  int *old_values   last values 
  int *init         set if old_values array should be initialized
  int range         the number of old values that will be averaged
*/
int average(int newest, int *old_values, int *init, int range)
{
  // initialize if needed
  if(*init)
  {
    for(int i = 0; i < range; i++)
    {
      old_values[i] = newest;
    }
    (*init) = 0;
  }
  // refresh array
  for(int i = range - 1; i > 0; i--)
  {
    old_values[i] = old_values[i-1];
  }
  old_values[0] = newest;
  // average over the array of old values
  int sum = 0;
  for(int i = 0; i < range; i++)
  {
    sum += old_values[i];
  }
  return sum / range;
}

/*
  obstacle_detection
  Searches the image for a specific obstacle.
  The image must be a grayscale.
  obstacle_detection implements a search automaton, 
  that searches in a horizontal line for a white area, followed 
  by a black area, followed by a second white area.
  This way we can identify our custom made obstacles.

  cv::Mat image   A grayscaled image
  int height      The highest point on the y axis that can be scanned (higher point = lower number)
  int index       Index in the y_coords_right array which is used for creating a debug view
  int* obstacle   A pointer to the array that wild store the start and the end for the obstacle
  int x_offset    The point on the x axis from where the search begins
*/
void obstacle_detection(cv::Mat image, int height, int index, int* obstacle, int x_offset)
{
  int state = OD_FIND_RIGHT_LINE;
  int x_right_od = NO_OBSTACLE;
  int x_left_od = NO_OBSTACLE;
  for( int y = 150; y > height; y-- ) 
  {
    state = OD_FIND_RIGHT_LINE;
    for( int x = x_offset; x > 10; x-- ) 
    {
      switch (state)
      {
        case OD_FIND_RIGHT_LINE:
          // found a white area if there are:
          // four consecutive pixel with a combined value of more than 60
          if(image_regional_value_check(image, x, y, 4, 60, MORE))
          {
            x_right_od = x - 5;
            y_coords_right[index] = y;
            state = OD_FIND_VOID;
          }
          break;
        case OD_FIND_VOID:
          // found a black area if there are:
          // five consecutive pixel with a combined value of less than 30
          if(image_regional_value_check(image, x, y, 5, 30, LESS))
          {
            state = OD_FIND_LEFT_LINE;
          }
          break;
        case OD_FIND_LEFT_LINE:
          if(image_regional_value_check(image, x, y, 4, 60, MORE))
          {
            x_left_od = x - 5;
            obstacle[0] = x_right_od;
            obstacle[1] = x_left_od;
            return;
          }
          break;
      }
    }
  }
  obstacle[0] = x_right_od; // default value, no obstacle detected
  obstacle[1] = x_left_od;
  return;
}

// receive and process images
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    // #####################################################################################################
    // ##### Save the image to a cv::Mat for further processing                               ##############
    // #####################################################################################################
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
    cv::inRange(crop, cv::Scalar(57, 25, 148), cv::Scalar(85, 178, 255), hsv_filtered);//62,25,148
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
    int x_right_line = find_main_right_point(blur, 150, 0);
    int x_left_line = find_main_left_point(blur, 150, 0);
    // average with the last couple of points to mitigate hickups
    x_now_right = average(x_right_line, x_greenlines_right, &x_right_init, RANGE_OF_AVERAGE);
    x_now_left = average(x_left_line, x_greenlines_left, &x_left_init, RANGE_OF_AVERAGE);
    // find two more points to differentiate between curves and straights 
    x_right_line_2 = find_secondary_right_point(blur, 115, 1);
    x_left_line_2 = find_secondary_left_point(blur, 115, 1);
    x_right_line_3 = find_secondary_right_point(blur, 50, 2);
    x_left_line_3 = find_secondary_left_point(blur, 50, 2);
    // mark the points as set 
    is_set = 1;
    // mark the points as updated
    new_points = 1;
    // #####################################################################################################
    // #####  Obstacle detection                                                              ##############
    // #####################################################################################################
    obstacle_detection(hsv_filtered_or, 100, 3, x_obstacle,hsv_filtered_or.cols);
    // #####################################################################################################
    // #####  Visualize for debug                                                             ##############
    // #####################################################################################################
    ROS_INFO("left:\t%i \tright: \t%i",x_now_left,x_now_right);
    // some colors for debug displays
    cv::Scalar red = cv::Scalar( 0, 0, 255 );
    cv::Scalar green = cv::Scalar( 0, 255, 0 );
    cv::Scalar orange = cv::Scalar( 0, 165, 254 );
    cv::Scalar blue = cv::Scalar( 255, 0, 0 );
    // convert line recognition image back to BGR to easily display it
    cv::Mat bgr  = cv::Mat::zeros( hsv_filtered.size() * 3, CV_8UC3 );
    cv::cvtColor(blur, bgr,  cv::COLOR_GRAY2BGR, 3);
    // display the points on the line and the points derived from obstacle detection
    cv::rectangle( bgr, cv::Point(x_right_line,y_coords_right[0]), cv::Point(x_right_line,y_coords_right[0]) + cv::Point( 2,2 ), green, 2 );
    cv::rectangle( bgr, cv::Point(x_left_line,y_coords_left[0]), cv::Point(x_left_line,y_coords_left[0]) + cv::Point( 2,2 ), red, 2 );
    cv::rectangle( bgr, cv::Point(x_right_line_2,y_coords_right[1]), cv::Point(x_right_line_2,y_coords_right[1]) + cv::Point( 2,2 ), green, 2 );
    cv::rectangle( bgr, cv::Point(x_left_line_2,y_coords_left[1]), cv::Point(x_left_line_2,y_coords_left[1]) + cv::Point( 2,2 ), red, 2 );
    cv::rectangle( bgr, cv::Point(x_right_line_3,y_coords_right[2]), cv::Point(x_right_line_3,y_coords_right[2]) + cv::Point( 2,2 ), green, 2 );
    cv::rectangle( bgr, cv::Point(x_left_line_3,y_coords_left[2]), cv::Point(x_left_line_3,y_coords_left[2]) + cv::Point( 2,2 ), red, 2 );
    cv::rectangle( bgr, cv::Point(x_obstacle[0],y_coords_right[3]), cv::Point(x_obstacle[0],y_coords_right[3]) + cv::Point( 2,2 ), orange, 2 );
    cv::rectangle( bgr, cv::Point(x_obstacle[1],y_coords_right[3]), cv::Point(x_obstacle[1],y_coords_right[3]) + cv::Point( 2,2 ), orange, 2 );
    // convert obstacle detection image back to BGR to easily display it
    cv::Mat bgr_or  = cv::Mat::zeros( hsv_filtered_or.size() * 3, CV_8UC3 );
    cv::cvtColor(hsv_filtered_or, bgr_or,  cv::COLOR_GRAY2BGR, 3);
    // display the points derived from obstacle detection
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
  ros::init(argc, argv, "line_recognition_node");
  // get ros node handle
  ros::NodeHandle nh;
  // create a window the show images for debug
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
  ros::Publisher right_pub = nh.advertise<std_msgs::Int32>("/line_recognition/right", 1);
  ros::Publisher left_pub = nh.advertise<std_msgs::Int32>("/line_recognition/left", 1);
  ros::Publisher right_pub_2 = nh.advertise<std_msgs::Int32>("/line_recognition/right_2", 1);
  ros::Publisher left_pub_2 = nh.advertise<std_msgs::Int32>("/line_recognition/left_2", 1);
  ros::Publisher right_pub_3 = nh.advertise<std_msgs::Int32>("/line_recognition/right_3", 1);
  ros::Publisher left_pub_3 = nh.advertise<std_msgs::Int32>("/line_recognition/left_3", 1);
  ros::Publisher obstacle_pub_right = nh.advertise<std_msgs::Int32>("/line_recognition/obstacle/right", 1);
  ros::Publisher obstacle_pub_left = nh.advertise<std_msgs::Int32>("/line_recognition/obstacle/left", 1);
  // Loop starts here:
  ROS_INFO("Line Recognition Start");
  /*
  ROS_INFO("\n
         /\\        ||     ||   ||==\\\\     //===\\\\          \n
        //\\\\       ||     ||   ||   \\\\   //     \\\\         \n
       //  \\\\      ||     ||   ||   ||   ||     ||             \n
      //==  \\\\     ||     ||   ||   ||   ||     ||          \n
     //      \\\\    \\\\     //   ||   //   \\\\    //            \n
    //        \\\\    \\\\===//    ||==//     \\\\==//                 \n");
  */
  ROS_INFO("\n       /\\        ||     ||   ||==\\\\     //===\\\\          \n      //\\\\       ||     ||   ||   \\\\   //     \\\\         \n     //  \\\\      ||     ||   ||   ||   ||     ||             \n    //==  \\\\     ||     ||   ||   ||   ||     ||          \n   //      \\\\    \\\\     //   ||   //   \\\\    //            \n  //        \\\\    \\\\===//    ||==//     \\\\==//                 \n");
  // loop rate value is set in Hz
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    if( is_set && new_points){
      // only try to publish points if they where initialized and are marked as updated
      new_points = 0;
      // create messages
      std_msgs::Int32 x_right;
      std_msgs::Int32 x_left;
      std_msgs::Int32 x_right_2;
      std_msgs::Int32 x_left_2;
      std_msgs::Int32 x_right_3;
      std_msgs::Int32 x_left_3;
      std_msgs::Int32 obstacle_right;
      std_msgs::Int32 obstacle_left;
      // fill the messages with data  
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
    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }
  ros::spin();
}