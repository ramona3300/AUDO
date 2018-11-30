#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/tf.h>
#include <signal.h>

double roll, pitch, yaw, last_t, t;
std_msgs::Int16 motor, steering;
bool stop = false;

void odomCallback(nav_msgs::Odometry::ConstPtr odomMsg, nav_msgs::Odometry* odom)
{
    *odom = *odomMsg;

    //Conversion to euler angles
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
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

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
