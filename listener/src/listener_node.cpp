#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard: %s", msg->data.c_str());
}

void imageCallback(const sensor_msgs::Image::ConstPtr& img){
    
    ROS_INFO("I heard Images");
    //ROS_INFO("I heard: %v", img->data);
}

int main( int argc, char **argv){
    ros::init(argc, argv, "listener");
    
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("chatter", 10, chatterCallback);
    ros::Subscriber subKinect = nh.subscribe("kinect2/qhd/image_color", 10, imageCallback);
    ros::spin();return 0;
}
