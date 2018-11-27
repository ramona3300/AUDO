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
static const std::string OPENCV_RAW = "Image RAAAAAWW window";
static const std::string OPENCV_STRANGE = "Image filtered window";

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

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
    ROS_INFO("Encoding: %s", enco);
    */

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
    /*
    outputQuad[0] = cv::Point2f( 1,1 ); //topleft mapped
    outputQuad[1] = cv::Point2f( 1279,719 ); //topright mapped
    outputQuad[2] = cv::Point2f( 750,719 ); //bottomright mapped
    outputQuad[3] = cv::Point2f( 600,719  ); //bottomleft mapped
    */
    outputQuad[0] = cv::Point2f( 1,1 ); //topleft mapped
    outputQuad[1] = cv::Point2f( 1279,1 ); //topright mapped
    outputQuad[2] = cv::Point2f( 800,719 ); //bottomright mapped
    outputQuad[3] = cv::Point2f( 300,719  ); //bottomleft mapped

    // Lambda Matrix
    cv::Mat lambda;//( 2, 4, CV_32FC1 );
    //lambda = cv::Mat::zeros( (cv_ptr->image).size(), (cv_ptr->image).type() );
    // Get the Perspective Transform Matrix i.e. lambda 
    lambda = cv::getPerspectiveTransform( inputQuad, outputQuad );
    cv::Mat bird  = cv::Mat::zeros( (cv_ptr->image).size(), (cv_ptr->image).type() );
    cv::warpPerspective(cv_ptr->image,bird,lambda,bird.size() );
    
    /* Set Region of Interest */

    int offset_x = 300;
    int offset_y = 0;

    cv::Rect roi;
    roi.x = offset_x;
    roi.y = offset_y;
    roi.width = bird.size().width - (offset_x*2);
    roi.height = bird.size().height - 150;

    /* Crop the original image to the defined ROI */

    //cv::Mat crop = bird(roi);
    
    cv::Mat brighton = cv::Mat::zeros((cv_ptr->image).size(), CV_8UC3);
    double alpha = 1.0; /*< Simple contrast control [1.0-3.0]*/
    int beta = 100;       /*< Simple brightness control [0-100]*/
    
    for( int y = 0; y < bird.rows; y++ ) {
        for( int x = 0; x < bird.cols; x++ ) {
            for( int c = 0; c < bird.channels(); c++ ) {
                brighton.at<cv::Vec3b>(y,x)[c] =
                  cv::saturate_cast<uchar>( alpha*bird.at<cv::Vec3b>(y,x)[c] + beta );
            }
        }
    }
    cv::Mat crop = brighton(roi);
    
    
    //ROS_INFO("BGR2HSV");
    //  cv::COLOR_BGR2HSV 
    cv::Mat hsv1  = cv::Mat::zeros( crop.size(), CV_8UC3 );
    cv::cvtColor(crop, hsv1, cv::COLOR_BGR2HSV, 3);

    cv::Mat hsv_filtered   = cv::Mat::zeros( hsv1.size(), CV_8UC3 );
    cv::inRange(hsv1, cv::Scalar(50, 20, 100), cv::Scalar(70, 255, 255), hsv_filtered);

    /*
    cv::Mat hsv2 = cv::Mat::zeros(brighton.size(), CV_8UC3);
    cv::cvtColor(brighton, hsv2, cv::COLOR_BGR2HSV, 3);

    cv::Mat hsv_original = cv::Mat::zeros(hsv2.size(), CV_8UC3);
    cv::inRange(hsv2, cv::Scalar(50, 20, 100), cv::Scalar(70, 255, 255), hsv_original);
    

    cv::imshow(OPENCV_WINDOW, hsv_original);
    cv::waitKey(3);
    */
    

    //ROS_INFO("AAHAHHAHAHAH");
    /*
    for( int y = 0; y < hsv_filtered.cols; y++ ) {
      for( int x = 0; x < hsv_filtered.rows; x++ ) {
        if(240 < hsv_filtered.at<uchar>(x,y)){
          hsv_filtered[x][y] = 1;
        }else{
          hsv_filtered[x][y] = 0;
        }
      }
    }
    */
    cv::Mat bin;
    //cv::threshold( hsv_filtered, bin, threshold_value, max_BINARY_value,threshold_type );
    cv::threshold( hsv_filtered, bin, 240, 255,CV_THRESH_BINARY );
    //cv::imshow(OPENCV_RAW, bin);
    //cv::waitKey(3);


    std::vector<cv::Vec4i> lines;
    //threshold: The minimum number of intersections to “detect” a line
    //minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
    //maxLineGap: The maximum gap between two points to be considered in the same line.
    //cv::HoughLinesP(bin, lines, 1, CV_PI/180, threshold, minLinLength, maxLineGap );
    cv::HoughLinesP(bin, lines, 1, CV_PI/180, 100, 100, 50 );

    ROS_INFO("lines:\t%d",lines.size());

    /*
    int last = 0;
    int index = 0;
    int length = ( (int) ( (bin.cols / 10) + 1 ) );
    cv::Point points_in_space[length];
    for( int y = 0; y < bin.rows; y++ ) {
        
        if(last + 10 < y){
          last = y;
        
          for( int x = 0; x < bin.cols; x++ ) {

            if(
              bin.at<uchar>(y,x)
              //&& 0 == hsv_filtered.at<uchar>(x+1,y)
              //&& 240 < hsv_filtered.at<uchar>(x+2,y)
              ){


            //if(240 < hsv_filtered.at<uchar>(x,y)){
              points_in_space[index] = cv::Point(x,y);
              index++;
              break;
            }
          }
            
        }
    }
    //ROS_INFO("BBEBEBEBEBEBEBEE");
    */
    cv::Mat bgr  = cv::Mat::zeros( hsv_filtered.size() * 3, CV_8UC3 );
    cv::cvtColor(hsv_filtered, bgr,  cv::COLOR_GRAY2BGR, 3);
    
    cv::Scalar color = cv::Scalar( 0, 0, 255 );
    /*
    int points_expected = (int)(hsv_filtered.rows / 10);
    ROS_INFO("ROWS / COLS = %d / %d",hsv_filtered.rows,hsv_filtered.cols);
    ROS_INFO("Points = %d / %d",index,points_expected);
    for( int i = 0; i < index; i++ ) {
      //ROS_INFO("i = %d",i);
      //cv::rectangle( bgr, points_in_space[i], points_in_space[i] + cv::Point( 1,1 ), color, 2 );
      
    }
    */
    for( int i = 0; i < lines.size(); i++ ) {
      cv::Vec4i vec = lines[i];
      line(bgr, cv::Point(vec[0],vec[1]), cv::Point(vec[2],vec[3]), color, 2, 8, 0);
    }
    //ROS_INFO("ROWS / COLS = %d / %d",bgr.rows,bgr.cols);
    cv::imshow(OPENCV_WINDOW, bgr);
    cv::waitKey(3);



    //cv::inRange(hsv, cv::Scalar(100, 20, 100), cv::Scalar(115, 255, 255), hsv_filtered);
    // H 107
    // S 255
    // V 224
    //cv::GaussianBlur( hsv_filtered, lowPassed, Size( 5, 5 ), 2, 2 );
   // cv::Mat lowPassed;
    //cv::bilateralFilter(hsv_filtered, lowPassed, 5, 120, 120);
    /*
    cv::resize(hsv_filtered, hsv_filtered, cv::Size(0,0), 0.5, 0.5, CV_INTER_LINEAR);
    cv::imshow(OPENCV_STRANGE, hsv_filtered);
    cv::waitKey(3);
    */
    /*
    //cv::Mat bgr; CV_32FC3
    // conversion fails do something about it
    //cv::Mat bgr  = cv::Mat::zeros( hsv_filtered.size(), CV_8UC3 );
    //cv::cvtColor(hsv_filtered, bgr, cv::COLOR_HSV2BGR, 3);

    // convert the image to grayscale
    cv::Mat gray = hsv_filtered;
    //cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
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
    // 10000 pixel²
    int rect_count = 100;
    int rect_count_actual = 0;
    std::vector<cv::Rect> boundRect_filtered( rect_count );
    for( size_t i = 0; i < rect_count; i++ )
    {
        if( boundRect[i].width * boundRect[i].height >= 500 ){
          boundRect_filtered[i] = boundRect[i];
          rect_count_actual++;
        }
    }


    // Create new Mat of unsigned 8-bit chars, filled with zeros.
    // It will contain all the drawings we are going to make (rects and circles).
    int type = CV_8UC3;
    //cv::Mat drawing = cv::Mat::zeros( canny_output.size(), type );
    cv::Mat drawing = crop;
    // For every contour: pick a random color, draw the contour, the bounding rectangle
    cv::RNG rng(12345);
    for( size_t i = 0; i< rect_count_actual; i++ )
    {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
        //cv::drawContours( drawing, contours_poly, (int)i, color );
        cv::rectangle( drawing, boundRect_filtered[i].tl(), boundRect_filtered[i].br(), color, 2 );
    }
    //cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
    //cv::rectangle( drawing, cv::Point( 0,0 ), cv::Point( 400,200 ), color, 2 );


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW,  drawing);
    cv::waitKey(3);
    */
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
  //cv::namedWindow(OPENCV_STRANGE);
  //cv::waitKey(3);
  //cv::namedWindow(OPENCV_RAW);
  //cv::waitKey(3);
  
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("cv_camera/image_raw", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("kinect2/qhd/image_color", 1, imageCallback);
  //image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);
  ///cv_camera/image_raw
  //kinect2/qhd/image_color
  
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




