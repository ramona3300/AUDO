#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <math.h>
#include <signal.h>

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

bool stop = false;
void mySiginthandler(int sig){
    stop = true;
}

double roll, pitch, yaw, last_t, t;
// current position
geometry_msgs::Point pos;
// position from when received current image
geometry_msgs::Point last_pos;
void odomCallback(nav_msgs::Odometry::ConstPtr odomMsg, nav_msgs::Odometry* odom)
{
    *odom = *odomMsg;

    //Conversion to euler angles
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);

    // get Position
    pos = odom->pose.pose.position;
    
}

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "testkreis_node", ros::init_options::NoSigintHandler);
  // get ros node handle
  ros::NodeHandle nh;
    signal(SIGINT, mySiginthandler);

  // sensor message container
  std_msgs::Int16 motor, steering;


  // generate control message publisher
    ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);
  ros::Publisher chatter = nh.advertise<std_msgs::String>("chatter",10);

  nav_msgs::Odometry odom;
  ros::Subscriber odomSub = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 10, boost::bind(odomCallback, _1, &odom));

  ROS_INFO("Test Kreis 123");

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(1);
  int count = 0;
  while (ros::ok())
  {
    ROS_INFO("Odom %i te Pose, x %f, y %f, z %f", count, pos.x, pos.y, pos.z);
      steering.data = 0;
      motor.data = 200;
      /*std_msgs::String msg;
   
       std::stringstream ss;
       ss << "Testkreis " << count;
       msg.data = ss.str();
       ROS_INFO("neue Nachricht");*/
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
//    chatter.publish(msg);
    count++;
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
