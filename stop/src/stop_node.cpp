#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>



int main(int argc, char** argv)
{
  // init this node
  ros::init(argc, argv, "stop_node");
  // get ros node handle
  ros::NodeHandle nh;

  // sensor message container
  std_msgs::Int16 motor, steering;

  // generate subscriber for sensor messages

  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);

  ROS_INFO("STOP");

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
      steering.data = 0;
      motor.data = 0;

    // publish command me0.5ssages on their topics
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
