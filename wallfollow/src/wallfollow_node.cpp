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
#define DS_CURVE 1 // curve mode
#define DS_STRAIGHT 2 // straight mode
#define DS_CURVE_AP 3 // straight mode
#define DS_STRAIGHT_AP 4 // curve mode
#define DS_STARTUP 5 // straight mode slow

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

int drive_state(int line_selection, int *curved, bool *actual_curve, int *straight_delay, int *curve_delay){
    bool is_curve = false;
    int curve_count = 0;
    double offset = 3;
    double diff = 0;
    // Curve or Straight?
    if(line_selection){// left
        diff = x_left_2 - x_left;
        if((x_left_3 - x_left_2 < diff + offset) && (x_left_3 - x_left_2 > diff - offset)){
            is_curve = false;// no curve
        }else{// curve
            is_curve = true;
        }
    }else{// right
        diff = x_right - x_right_2;
        if((x_right_2 - x_right_3 < diff + offset) && (x_right_2 - x_right_3 > diff - offset)){
            is_curve = false;// no curve
        }else{// curve
            is_curve = true;
        }
    }
    // is there really a curve
    int temp[30];
    for(int i = 0; i < 30; i++){temp[i] = curved[i];}
    for(int i = 1; i < 30; i++){
        curved[i] = temp[i - 1];
    }
    curved[0] = is_curve ? 1 : 0;
    for(int i = 0; i < 30; i++){
        curve_count += curved[i];
    }

    //ROS_INFO("count = %d curve = %d curve_del = %d straight_del = %d",curve_count,*actual_curve,*curve_delay,*straight_delay);

    // when approaching a curve wait for some time before switching to curve mode
    if(curve_count >= 16){
        // approaching a curve
        if(!(*actual_curve)){
            // delay actually being in the curve
            if(*straight_delay == 0){
                *actual_curve = true;
                *curve_delay = 20;
                return DS_CURVE;
            }else{
                (*straight_delay)--;
                return DS_CURVE_AP;
            }
        }else{
            *actual_curve = true;
            *curve_delay = 20;
            return DS_CURVE;
        }
    }else if(curve_count <= 15){
        // approaching a straight
        if(*actual_curve){
            // delay actually being in the straight
            if(*curve_delay == 0){
                *actual_curve = false;
                *straight_delay = 20;
                return DS_STRAIGHT;
            }else{
                (*curve_delay)--;
                return DS_STRAIGHT_AP;
            }
        }else{
            *actual_curve = false;
            *straight_delay = 20;
            return DS_STRAIGHT;
        }
    }
}

int current_steering steering_characteristic(int s_out_av, int last_steer){
    // do clever things
    return s_out_av;
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

  // subscribe to ultra sonic
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
  
  // variables for controlling
  // 0 = right; 1 = left
  int line_selection = 0;
  int motor_speed = 0;
  // PID values
  int pk = 3000.0 * 0.7;
  int pd = 1000.0 * 0.1;
  int pi = 200;
  // loop dependent variables
  int last_steer = 0; // last steering value
  double last_err = 0;
  double err = 0;
  double p_err = 0;
  double d_err = 0;
  double i_err = 0;
  double istwert = 0;
  double sollwert = 1.4;
  int s_out = 0;
  last_t = ((double)clock()/CLOCKS_PER_SEC);
  int s_out_ar[10] = {0,0,0,0,0,0,0,0,0,0};
  // drive_state variables
  int current_drive_state = DS_STARTUP;
  int curved[30];
  for(int i = 0; i < 30; i++){curved[i] = 0;}
  bool actual_curve = false;
  int straight_delay = 0;
  int curve_delay = 0;
  int startup_delay = 20;

  // #####################################################################################################
  // ##### Loop starts here:    loop rate value is set in Hz                                ##############
  // #####################################################################################################
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    t = ((double)clock()/CLOCKS_PER_SEC);
    s_out = 0;
    motor_speed = 400; // works with 300 
    
    // TODO do better
    if( x_left >= x_right - 10){// only one line
        line_selection = 0;// just choose the right line
    }

    // which line to follow
    if(line_selection){
        istwert = ( (double)x_left + 60.0 ) / 100.0;// left
    }else{
        istwert = (double)x_right / 100.0;// right
    }
    // #####################################################################################################
    // ##### Controlling                                                                      ##############
    // #####################################################################################################
    // get drive state
    current_drive_state = drive_state(line_selection, curved, &actual_curve, &straight_delay, &curve_delay);
    // ignore drive_state for some time after start
    if(startup_delay > 0){
        current_drive_state = DS_STARTUP;
        startup_delay--;
    }
    switch(current_drive_state){
        case DS_CURVE: // Curve mode        1
            motor_speed = (int) ( (double)motor_speed * 0.8 );
            pk = (int) (3000.0 * 0.7 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.1 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_CURVE_AP: // Straight mode  2
            motor_speed = (int) ( (double)motor_speed * 0.9 );
            pk = (int) (3000.0 * 0.15 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_STRAIGHT: // Straight mode  3
            motor_speed = (int) ( (double)motor_speed * 1.2 );
            pk = (int) (3000.0 * 0.15 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0 / (double)motor_speed ) );
            break;
        case DS_STRAIGHT_AP: // Curve mode  4
            motor_speed = (int) ( (double)motor_speed * 0.9 );
            pk = (int) (3000.0 * 0.3 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_STARTUP:// Straight mode slow  5
            motor_speed = 300;
            pk = (int) (3000.0 * 0.15 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
            break;
    }
    // Regelabweichung
    err = sollwert - istwert;
    p_err = pk * err;// P-Anteil
    d_err = pd * (err - last_err) / (t - last_t);// D-Anteil
    // calculate control value
    s_out = -(p_err +  d_err);
    // limit s_out to +-MAX_STEERING_ANGLE
    if(s_out > MAX_STEERING_ANGLE) s_out = MAX_STEERING_ANGLE;
    else if(s_out < -MAX_STEERING_ANGLE) s_out = -MAX_STEERING_ANGLE;

    // flatten s_out
    int temp[10];
    for(int i = 0; i < 10; i++){temp[i] = s_out_ar[i];}
    for(int i = 1; i < 10; i++){s_out_ar[i] = temp[i - 1];}
    s_out_ar[0] = s_out;
    int s_out_av = 0;
    for(int i = 0; i < 10; i++){
        s_out_av += s_out_ar[i] * 0.1 * (10 - i);
    }
    s_out_av = s_out_av / 5;
    // steering adjustments
    int current_steering = steering_characteristic(s_out_av, last_steer);
    last_steer = current_steering;
    steering.data = (int)s_out_av;
    ROS_INFO("State = %d, line = %d, steer = %d, speed = %d",current_drive_state, line_selection, s_out_av, motor_speed);
    // speed control and collision protection
    if (collision_protection((double)usf.range)){
      motor.data = 0;
    }else motor.data = motor_speed;
    
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
