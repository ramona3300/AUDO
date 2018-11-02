#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>

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

int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "testkreis_node");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  std_msgs::Int16 motor, steering;


  // generate control message publisher
    ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("A simple avoidance, just better!");

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
     
    // simple wall crash avoidance algorithm ..
    if (usr.range < 0.3 && usl.range >= 0.3)
    {
      steering.data = -750;
      motor.data = 500;
    }
    else if (usl.range < 0.3 && usr.range >= 0.3)
    {
      steering.data = 750;
      motor.data = 500;
    }
    else if (usl.range > 0.3 && usr.range > 0.3)
    {
      steering.data = 0;
      motor.data = 500;
    }
    else
    {
      steering.data = 0;
      motor.data = 0;
    }
     // publish command messages on their topics
    motorCtrl.publish(motor);
    steeringCtrl.publish(steering);
    // side note: setting steering and motor even though nothing might have
    // changed is actually stupid but for this demo it doesn't matter too much.

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
