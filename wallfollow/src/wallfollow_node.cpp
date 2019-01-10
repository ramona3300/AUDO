#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <tf/tf.h>
#include <signal.h>
// range of steering average for speed control
#define RANGE_OF_AVERAGE 10
// range of usf value average for collision protection
#define RANGE_OF_USF_AVERAGE 20
#define MAX_STEERING_ANGLE 700

double roll, pitch, yaw, last_t, t;
std_msgs::Int16 motor, steering;
bool stop = false;
double x_right;
double x_left;
double x_right_2;
double x_left_2;
double x_right_3;
double x_left_3;
int steering_history[RANGE_OF_AVERAGE];
bool steering_flag = true;
double usf_history[RANGE_OF_AVERAGE];
bool usf_flag = true;
bool is_curve = false;

int speed_control(int s_out){
    // initialization
    if(steering_flag){
      for(int i = 0; i < RANGE_OF_AVERAGE; i++){
        steering_history[i] = s_out;
      }
      steering_flag = false;
    }
    // refresh array 
    for(int i = 0; i < RANGE_OF_AVERAGE - 1; i++){
        steering_history[i + 1] = steering_history[i];
    }
    steering_history[0] = s_out;

    // average
    int average = 0;
    for(int i = 0; i < RANGE_OF_AVERAGE; i++){
        average += steering_history[i];
      }
    average = average / RANGE_OF_AVERAGE;

    // calculate speed
    return 400 - 150 * average / MAX_STEERING_ANGLE;
}

bool collision_protection(double range){
    // initialization
    if(usf_flag){
      for(int i = 0; i < RANGE_OF_USF_AVERAGE; i++){
        usf_history[i] = 0;
      }
      usf_flag = false;
      return true;
    }
    // refresh array 
    if(range > 0){
        for(int i = 0; i < RANGE_OF_USF_AVERAGE - 1; i++){
            usf_history[i + 1] = usf_history[i];
        }
        usf_history[0] = range;
    }

    // average
    double average = 0;
    for(int i = 0; i < RANGE_OF_USF_AVERAGE; i++){
        average += usf_history[i];
      }
    average = average / RANGE_OF_USF_AVERAGE;

    if(average < 0.3) return true;
    else return false;
}

void odomCallback(nav_msgs::Odometry::ConstPtr odomMsg, nav_msgs::Odometry* odom)
{
    *odom = *odomMsg;

    //Conversion to euler angles
    tf::Quaternion q;
    tf::quaternionMsgToTF(odom->pose.pose.orientation, q);
    tf::Matrix3x3 mat(q);
    mat.getEulerYPR(yaw, pitch, roll);
}
// receive right line
void right_Callback(std_msgs::Int32::ConstPtr msg, double* data)
{
  *data = (double) msg->data;
}
// receive left line
void left_Callback(std_msgs::Int32::ConstPtr msg, double* data)
{
  *data = (double) msg->data;
}

// receive right line
void right_Callback_2(std_msgs::Int32::ConstPtr msg, double* data)
{
  *data = (double) msg->data;
}
// receive left line
void left_Callback_2(std_msgs::Int32::ConstPtr msg, double* data)
{
  *data = (double) msg->data;
}

// receive right line
void right_Callback_3(std_msgs::Int32::ConstPtr msg, double* data)
{
  *data = (double) msg->data;
}
// receive left line
void left_Callback_3(std_msgs::Int32::ConstPtr msg, double* data)
{
  *data = (double) msg->data;
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
  ros::Subscriber right_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/right", 1, boost::bind(right_Callback, _1, &x_right));
  ros::Subscriber left_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/left", 1, boost::bind(left_Callback, _1, &x_left));
  ros::Subscriber right_sub_2 = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/right_2", 1, boost::bind(right_Callback, _1, &x_right_2));
  ros::Subscriber left_sub_2 = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/left_2", 1, boost::bind(left_Callback, _1, &x_left_2));
  ros::Subscriber right_sub_3 = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/right_3", 1, boost::bind(right_Callback, _1, &x_right_3));
  ros::Subscriber left_sub_3 = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/left_3", 1, boost::bind(left_Callback, _1, &x_left_3));
  
  // generate control message publisher
  ros::Publisher motorCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_motor_level_msg", 1);
  ros::Publisher steeringCtrl =
      nh.advertise<std_msgs::Int16>("/uc_bridge/set_steering_level_msg", 1);


 

  ROS_INFO("A simple Wallfollow");

  // 0 = right; 1 = left
  int line_selection = 0;
  int pk = 3000 * 0.7;
  int pd = 1000 * 0.1;
  int pi = 200;

  int pk_straight = 3000 * 0.15;
  int pd_straight = 1000 * 0.05;
  int pi_straight = 200;
  

  double last_err = 0;
  double err = 0;
  double p_err = 0;
  double d_err = 0;
  double i_err = 0;
  double istwert = 0;
  last_t = ((double)clock()/CLOCKS_PER_SEC);
  int s_out_ar[10] = {0,0,0,0,0,0,0,0,0,0};

  int curved[30] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  bool actual_curve = false;
  int curve_count = 0;

  // Loop starts here:
  // loop rate value is set in Hz
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    t = ((double)clock()/CLOCKS_PER_SEC);
    int s_out = 0;
    // TODO stimmt das noch
    double sollwert = 1.4;

    if( x_left >= x_right - 10){
        // only one line
        // just choose the right line
        line_selection = 0;
    }

    if(line_selection){
        // left
        istwert = ( (double)x_left + 60.0 ) / 100.0;
    }else{
        // right
        istwert = (double)x_right / 100.0;
    }
    
    double offset = 3;
    double diff = 0;
    // Curve or Straight?
    if(line_selection){
        // left
         diff = x_left_2 - x_left;

        // no curve
        if((x_left_3 - x_left_2 < diff + offset) && (x_left_3 - x_left_2 > diff - offset)){
            is_curve = false;
        }
        // curve
        else{
            is_curve = true;
        }
        

    }else{
        // right
        diff = x_right - x_right_2;

        // no curve
        if((x_right_2 - x_right_3 < diff + offset) && (x_right_2 - x_right_3 > diff - offset)){
            is_curve = false;
        }
        // curve
        else{
            is_curve = true;
        }
    }
    // is there really a curve
    for(int i = 1; i < 30; i++){
        curved[i] = curved[i - 1];
    }
    curved[0] = is_curve ? 1 : 0;
    curve_count = 0;
    for(int i = 0; i < 30; i++){
        curve_count += curved[i];
    }
    if(curve_count >= 25){
        actual_curve = true;
    }else if(curve_count <= 15){
        actual_curve = false;
    }

    
    // #####################################################################################################
    // ##### Controlling                                                                      ##############
    // #####################################################################################################
    
    // Regelabweichung
    err = sollwert - istwert;

    if(actual_curve){
        // P-Anteil
        p_err = pk * err;
        // D-Anteil
        d_err = pd * (err - last_err) / (t - last_t);
        // I_Anteil mit Anti-Windup
        i_err += pi*err;
    }else{
        // P-Anteil
        p_err = pk_straight * err;
        // D-Anteil
        d_err = pd_straight * (err - last_err) / (t - last_t);
        // I_Anteil mit Anti-Windup
        i_err += pi_straight * err;
    }
    
    if(i_err > 400) i_err = 400;
    else if(i_err < -400) i_err = -400;

    // calculate control value
    s_out = -(p_err +  d_err /*+ i_err*/);
    
    // limit s_out to +-MAX_STEERING_ANGLE
    if(s_out > MAX_STEERING_ANGLE) s_out = MAX_STEERING_ANGLE;
    else if(s_out < -MAX_STEERING_ANGLE) s_out = -MAX_STEERING_ANGLE;

    //ROS_INFO("s_out: %d \t d_err: %f", s_out, d_err);
    for(int i = 1; i < 10; i++){
        s_out_ar[i] = s_out_ar[i - 1];
    }
    s_out_ar[0] = s_out;

    int s_out_av = 0;
    for(int i = 0; i < 10; i++){
        s_out_av += s_out_ar[i] * 0.1 * (10 - i);
    }
    s_out_av = s_out_av / 5;
    ROS_INFO("Istwert = %f Curve = %d, line_sel = %d, steer = %d",istwert, actual_curve, line_selection, s_out_av);

    steering.data = (int)s_out_av;

    // speed control and collision protection
    if (collision_protection((double)usf.range))
    {
      motor.data = 0;
    }
    else motor.data = 300 /*speed_control((int)abs(s_out))*/;


    //ROS_INFO("speed: %d", motor.data);

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
    
    last_err = err;
    last_t = t;

    // clear input/output buffers
    ros::spinOnce();
    // this is needed to ensure a const. loop rate
    loop_rate.sleep();
  }

  ros::spin();
}
