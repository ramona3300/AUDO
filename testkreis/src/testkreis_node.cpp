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
  ros::Publisher chatter = nh.advertise<std_msgs::String>("chatter",10);

  ROS_INFO("Test Kreis 123");

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(0.5);
  int count = 0;
  while (ros::ok())
  {
      steering.data = 700;
      motor.data = 200;
      std_msgs::String msg;
   
       std::stringstream ss;
       ss << "Testkreis " << count;
       msg.data = ss.str();

    // publish command messages on their topics
    motorCtrl.publish(motor);
    steeringCtrl.publish(steering);
    chatter.publish(msg);
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
