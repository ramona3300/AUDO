#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/tf.h>
#include <signal.h>

double roll, pitch, yaw, last_t, t;
std_msgs::Int16 motor, steering;
bool stop = false;
// current position
geometry_msgs::Point pos;
// position from when received current image
geometry_msgs::Point last_pos;
// received new image
int new_image = 1;

int x_1;
int y_1;
int x_2;
int y_2;

// return the distance between AUDO and line in cm
int get_distance(){
    int current_x = pos.x;
    int current_y = pos.y;
    int last_x = last_pos.x;
    int last_y = last_pos.y;
    // calculate y offset from last_pos
    int y_offset = current_y - last_y;
    // calculate distance with offset with help of m
    double m = ( (double)(x_2) - (double)(x_1) ) / ( (double)(y_2) - (double)(y_1) );
    // where is the line now?
    int y_offset_in_px = 0; // = ???

    int x_now = x_1 - y_offset_in_px * m;
    // x_now is the distance in pixels from the line
    // x_now should be regualated to be 260 at all times
    return x_now;
}


void odomCallback(nav_msgs::Odometry::ConstPtr odomMsg, nav_msgs::Odometry* odom)
{
    *odom = *odomMsg;

    //Conversion to euler angles
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);

    // set image received position
    if( new_image ){
        last_pos = odom->pose.pose.position;
        new_image = 0;
    }
    // get Position
    pos = odom->pose.pose.position;
    
}

void x1_Callback(std_msgs::Int32::ConstPtr msg, int* data)
{
  // inform odomCallback that a new image arrived
  new_image = 1;
  *data = msg->data;
}
void y1_Callback(std_msgs::Int32::ConstPtr msg, int* data)
{
  *data = msg->data;
}
void x2_Callback(std_msgs::Int32::ConstPtr msg, int* data)
{
  *data = msg->data;
}
void y2_Callback(std_msgs::Int32::ConstPtr msg, int* data)
{
  *data = msg->data;
}


// gets called whenever a new message is availible in the input puffer
void uslCallback(sensor_msgs::Range::ConstPtr uslMsg, sensor_msgs::Range* usl)
{
  *usl = *uslMsg;
}

// gets called whenever a new message is availible in the input puffer
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

// gets called whenever a new message is availible in the input puffer
void usrCallback(sensor_msgs::Range::ConstPtr usrMsg, sensor_msgs::Range* usr)
{
  *usr = *usrMsg;
}
void mySiginthandler(int sig){
    stop = true;
}

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "wallfollow_node", ros::init_options::NoSigintHandler);
  // get ros node handle
  ros::NodeHandle nh;
  signal(SIGINT, mySiginthandler);

  // sensor message container
  std_msgs::Int16 motor, steering;
  sensor_msgs::Range usr, usf, usl;
  nav_msgs::Odometry odom;


  ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 10, boost::bind(odomCallback, _1, &odom));
  ros::Subscriber usrSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usr", 10, boost::bind(usrCallback, _1, &usr));
  ros::Subscriber uslSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usl", 10, boost::bind(uslCallback, _1, &usl));
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));
  
  // subscribe to the line_recoqnition
  ros::Subscriber x1_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/x1", 1, boost::bind(x1_Callback, _1, &x_1));
  ros::Subscriber y1_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/y1", 1, boost::bind(y1_Callback, _1, &y_1));
  ros::Subscriber x2_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/x2", 1, boost::bind(x2_Callback, _1, &x_2));
  ros::Subscriber y2_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/y2", 1, boost::bind(y2_Callback, _1, &y_2));
  


  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("A simple Wallfollow");

  double sollwert = 0.4;
  int pd = 200;
  int pk = 800;
  double last_err = 0;
  double err = 0;
  double d_err = 0;
  last_t = ((double)clock()/CLOCKS_PER_SEC);

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(15);
  while (ros::ok())
  {
    t = ((double)clock()/CLOCKS_PER_SEC);
    int s_out = 0;

    // Regelabweichung
    err = sollwert - usr.range*cos(yaw);

    d_err = (err-last_err);

    s_out = -(pk * err + pd * d_err / (t - last_t) );
    if(s_out > 600) s_out = 600;
    else if(s_out < -600) s_out = -600;


    ROS_INFO("s_out: %d \t d_err: %f", s_out, d_err);

    steering.data = (int)s_out;

    if (usf.range < 0.45)
    {
      motor.data = 0;
    }
    else motor.data = 300;
    if (stop){
        ROS_INFO("Stop Request send");
        motor.data = 0;
        steering.data = 0;
        motorCtrl.publish(motor);
        steeringCtrl.publish(steering);
        ros::shutdown();
    }
     // publish command messages on their topics
    motorCtrl.publish(motor);
    steeringCtrl.publish(steering);
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.
    
    last_err = err;
    last_t = t;

    // TODO:
    // get values from odom.pose.pose.position (and odom.twist.twist.linear.x an odom.twist.twist.linear.y)
    // if new image from contour_detection is received
    //  then:   save position of audo, save line to follow
    //          reset position = 0
    // if  wallfollow wants to calc new value and no new image is available
    //  then:   use odomerty position for calculation until next image is received


    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
