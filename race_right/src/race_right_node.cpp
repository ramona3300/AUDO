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
#define MAX_MOTOR_SPEED 1000
#define MIN_MOTOR_SPEED 0
// driving states
#define DS_CURVE 1 // curve mode
#define DS_STRAIGHT 2 // straight mode
#define DS_CURVE_AP 3 // curve approach
#define DS_STRAIGHT_AP 4 // straight approach
#define DS_STARTUP 5 // startup mode (straight mode slow)
#define DS_STOP 13 // Stop


#define RANGE_OF_STEERING_AVG 20 // for s_out lowpassfilter
#define RANGE_OF_STEERING_AVG_RACE 8 // for s_out lowpassfilter works with 10


double last_t, t;
std_msgs::Int16 motor, steering;
bool stop = false;
double x_right;
double x_left;
double x_right_2;
double x_left_2;
double x_right_3;
double x_left_3;


double usf_history[RANGE_OF_AVERAGE];
bool usf_flag = true;


/*
  weighted_average
  Averages the last couple of integers, given an array of the last integers
  and a range to average over.
  It weights the values in the array based on their index, higher index less weight
  It also can initialize the array if needed.

  int newest        newest value 
  int *old_values   last values 
  int *init         set if old_values array should be initialized
  int range         the number of old values that will be averaged
*/
int weighted_average(int newest, int *old_values, int *init, int range)
{
  // initialize if needed
  if(*init)
  {
    for(int i = 0; i < range; i++)
    {
      old_values[i] = newest;
    }
    (*init) = 0;
  }
  // refresh array
  for(int i = range - 1; i > 0; i--)
  {
    old_values[i] = old_values[i-1];
  }
  old_values[0] = newest;
  // average over the array of old values
  double sum = 0;
  double weight_sum = 0;
  double weight = 0;
  for(int i = 0; i < range; i++)
  {
    weight = ( range - i );
    weight_sum += weight;
    sum += ( (double) old_values[i] ) * weight;
  }
  return (int) ( sum  / weight_sum );
}

/*
    lines_recognized
    Checks which lines have been successfully recognized
    return	1	Both
            2	Only Right 
            3 Only Left	
*/
int lines_recognized(int line_selection)
{
  if(	x_left > x_right - 5 ||
      x_left + 5 > x_right)
  {
    // Only One line has been recognized
    // The recognized line is the one that is on the side that we drive
    if(line_selection)
    {
      return 3;
    }
    else
    {
      return 2;
    }
  }
  return 1;
}
/*
  collision_protection

  double range
*/
bool collision_protection(double range)
{
  // initialization
  if(usf_flag)
  {
    for(int i = 0; i < RANGE_OF_USF_AVERAGE; i++)
    {
      usf_history[i] = 0;
    }
    usf_flag = false;
    return true;
  }
  // refresh array 
  if(range > 0)
  {
    for(int i = 0; i < RANGE_OF_USF_AVERAGE - 1; i++)
    {
      usf_history[i + 1] = usf_history[i];
    }
      usf_history[0] = range;
  }
  // average
  double average = 0;
  for(int i = 0; i < RANGE_OF_USF_AVERAGE; i++)
  {
    average += usf_history[i];
  }
  average = average / RANGE_OF_USF_AVERAGE;
  if(average < 0.35) 
  {
    return true;
  }
  else
  {
    return false;
  }
}
/*
  drive_state
  Determines drive state by detecting wether the road is curved or straight

  line_selection      Is car following the right or the left line
  *curved             array that stores past detections
  *actual_curve       is there really a curve?
  *straight_delay     delay used to ensure straight mode a little longer 
  *curve_delay        delay used to ensure curve mode a little longer
*/
int drive_state(int line_selection, int *curved, bool *actual_curve, int *straight_delay, int *curve_delay){
  bool is_curve = false;
  int curve_count = 0;
  double offset = 3;
  double diff = 0;
  double diff_2 = 0;
  // Curve or Straight?
  if(line_selection)
  {// left
    diff = x_left_2 - x_left;
    diff_2 = x_left_3 - x_left_2;
    if( (diff_2 < diff + offset) && 
        (diff_2 > diff - offset))
    {
      is_curve = false;// no curve
    }
    else
    {// curve
      is_curve = true;
    }
  }
  else
  {// right
    diff = x_right - x_right_2;
    diff_2 = x_right_2 - x_right_3;
    if( (diff_2 < diff + offset) && 
            (diff_2 > diff - offset))
      {
        is_curve = false;// no curve
      }
      else
      {// curve
        is_curve = true;
      }
  }
  // is there really a curve
  for(int i = 9; i > 0; i--)
  {
    curved[i] = curved[i - 1];
  }
  curved[0] = is_curve ? 1 : 0;
  for(int i = 0; i < 10; i++)
  {
    curve_count += curved[i];
  }
  //ROS_INFO("count = %d curve = %d curve_del = %d straight_del = %d, diff = %f, diff_2 = %f",curve_count,*actual_curve,*curve_delay,*straight_delay, diff, diff_2);
  //ROS_INFO("x_right = %f x_left = %f x_right_2 = %f x_left_2 = %f x_right_3 = %f x_left_3 = %f",x_right, x_left, x_right_2, x_left_2, x_right_3, x_left_3);
  // when approaching a curve wait for some time before switching to curve mode
  ROS_INFO("----------------------------------------------------------------------------count = %d",curve_count);
  if(curve_count > 4){
    // approaching a curve
    if(!(*actual_curve))
    {
      // delay actually being in the curve
      if(*straight_delay == 0)
      {
        *actual_curve = true;
        *curve_delay = 10;
        return DS_CURVE;
      }
      else
      {
        (*straight_delay)--;
        return DS_CURVE_AP;
      }
    }
    else
    {
      *actual_curve = true;
      *curve_delay = 10;
      return DS_CURVE;
    }
  }
  else
  {
    // approaching a straight
    if(*actual_curve)
    {
      // delay actually being in the straight
      if(*curve_delay == 0)
      {
        *actual_curve = false;
        *straight_delay = 10;
        return DS_STRAIGHT;
      }
      else
      {
        (*curve_delay)--;
        return DS_STRAIGHT_AP;
      }
    }
    else
    {
      *actual_curve = false;
      *straight_delay = 10;
      return DS_STRAIGHT;
    }
  }
}

// receive and store an INT32 as double
void INT32_Callback_as_double(std_msgs::Int32::ConstPtr msg, double* data)
{
  *data = (double) msg->data;
}

// receive and store an INT32 as integer
void INT32_Callback_as_int(std_msgs::Int32::ConstPtr msg, int* data)
{
  *data = (int) msg->data;
}

// gets called whenever a new message is availible in the input puffer
void usfCallback(sensor_msgs::Range::ConstPtr usfMsg, sensor_msgs::Range* usf)
{
  *usf = *usfMsg;
}

void mySiginthandler(int sig){
    stop = true;
}

int main(int argc, char** argv)
{
  ROS_INFO("Race Mode Right Start");
  /*
  ROS_INFO("\n
         /\\        ||     ||   ||==\\\\     //===\\\\          \n
        //\\\\       ||     ||   ||   \\\\   //     \\\\         \n
       //  \\\\      ||     ||   ||   ||   ||     ||             \n
      //==  \\\\     ||     ||   ||   ||   ||     ||          \n
     //      \\\\    \\\\     //   ||   //   \\\\    //            \n
    //        \\\\    \\\\===//    ||==//     \\\\==//                 \n");
  */
  ROS_INFO("\n       /\\        ||     ||   ||==\\\\     //===\\\\          \n      //\\\\       ||     ||   ||   \\\\   //     \\\\         \n     //  \\\\      ||     ||   ||   ||   ||     ||             \n    //==  \\\\     ||     ||   ||   ||   ||     ||          \n   //      \\\\    \\\\     //   ||   //   \\\\    //            \n  //        \\\\    \\\\===//    ||==//     \\\\==//                 \n");
  
  // init this node
  ros::init(argc, argv, "race_right_node", ros::init_options::NoSigintHandler);
  // get ros node handle
  ros::NodeHandle nh;
  signal(SIGINT, mySiginthandler);
  // sensor message container
  std_msgs::Int16 motor, steering;
  sensor_msgs::Range usr, usf, usl;
  // subscribe to ultra sonic
  ros::Subscriber usfSub = nh.subscribe<sensor_msgs::Range>(
      "/uc_bridge/usf", 10, boost::bind(usfCallback, _1, &usf));
  // subscribe to the line_recoqnition
  ros::Subscriber right_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recognition/right", 1, boost::bind(INT32_Callback_as_double, _1, &x_right));
  ros::Subscriber left_sub = nh.subscribe<std_msgs::Int32>(
      "/line_recognition/left", 1, boost::bind(INT32_Callback_as_double, _1, &x_left));
  ros::Subscriber right_sub_2 = nh.subscribe<std_msgs::Int32>(
      "/line_recognition/right_2", 1, boost::bind(INT32_Callback_as_double, _1, &x_right_2));
  ros::Subscriber left_sub_2 = nh.subscribe<std_msgs::Int32>(
      "/line_recognition/left_2", 1, boost::bind(INT32_Callback_as_double, _1, &x_left_2));
  ros::Subscriber right_sub_3 = nh.subscribe<std_msgs::Int32>(
      "/line_recognition/right_3", 1, boost::bind(INT32_Callback_as_double, _1, &x_right_3));
  ros::Subscriber left_sub_3 = nh.subscribe<std_msgs::Int32>(
      "/line_recognition/left_3", 1, boost::bind(INT32_Callback_as_double, _1, &x_left_3));
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
  double sollwert = 1.8;
  int s_out = 0;
  last_t = ((double)clock()/CLOCKS_PER_SEC);
  int s_out_ar[RANGE_OF_STEERING_AVG];
  int s_out_init = 1;
  //for(int i = 0; i < RANGE_OF_STEERING_AVG; i++){s_out_ar[i] = 0;}
  // drive_state variables
  int current_drive_state = DS_STARTUP;
  int curved[10];
  for(int i = 0; i < 10; i++)
  {
    curved[i] = 0;
  }
  bool actual_curve = true;
  int straight_delay = 10;
  int curve_delay = 10;
  int startup_delay = 10;
  int s_out_av = 0;


  // #####################################################################################################
  // ##### Loop starts here:    loop rate value is set in Hz                                ##############
  // #####################################################################################################
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    t = ((double)clock()/CLOCKS_PER_SEC);
    s_out = 0;
    motor_speed = 500; // works with 300 for obstacle and 600 for race
    
    // which line to follow
    if(line_selection)
    {
      istwert = ( (double)x_left + 65.0 ) / 100.0;// left
    }
    else
    {
      istwert = (double)x_right / 100.0;// right
    }
    // #####################################################################################################
    // ##### Controlling                                                                      ##############
    // #####################################################################################################
    // get drive state and obstacle state
    current_drive_state = drive_state(line_selection, curved, &actual_curve, &straight_delay, &curve_delay);


    // ignore drive_state for some time after start
    if(startup_delay > 0)
    {
      current_drive_state = DS_STARTUP;
      startup_delay--;
    }
    int recognized = lines_recognized(line_selection);
    // set controller values and motor speed according to drive state
    switch(current_drive_state)
    {
      // ####################### Race Mode ################################## Mode Detail ### No ###
      case DS_CURVE: //                                                   ### Curve       ###  1 ###
        motor_speed = (int) ( (double)motor_speed * 1.0 );
        pk = (int) (3000.0 * 0.7 * ( 300.0  / (double)motor_speed ) );
        pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
        break;
      case DS_CURVE_AP: //                                                ### Straight    ###  3 ###
        motor_speed = (int) ( (double)motor_speed * 1.0 );
        pk = (int) (3000.0 * 0.5 * ( 300.0  / (double)motor_speed ) );
        pd = (int) (1000.0 * 0.01 * ( 300.0  / (double)motor_speed ) );
        break;
      case DS_STRAIGHT: //                                                ### Straight    ###  2 ###
        motor_speed = (int) ( (double)motor_speed * 1.1 );
        pk = (int) (3000.0 * 0.1 * ( 300.0  / (double)motor_speed ) );
        pd = (int) (1000.0 * 0.01 * ( 300.0 / (double)motor_speed ) );
        break;
      case DS_STRAIGHT_AP: //                                             ### Curve       ###  4 ###
        motor_speed = (int) ( (double)motor_speed * 1.0 );
        pk = (int) (3000.0 * 0.2 * ( 300.0  / (double)motor_speed ) );
        pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
        break;
      // ####################### Startup Mode ############################### Mode Detail ### No ###
      case DS_STARTUP://                                                ### Straight slow ###  5 ###
        motor_speed = 300;
        pk = (int) (3000.0 * 0.15 * ( 300.0  / (double)motor_speed ) );
        pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
        break;
      case DS_STOP: //                                                  ### Stop driving  ### 13 ###
        motor_speed = 0;
        break;
    }
        // calculate controller values
        err = sollwert - istwert;
        p_err = pk * err;// P-portion
        d_err = pd * (err - last_err) / (t - last_t);// D-portion
        s_out = -(p_err +  d_err);
        // limit s_out to +-MAX_STEERING_ANGLE
        if(s_out > MAX_STEERING_ANGLE) 
        {
          s_out = MAX_STEERING_ANGLE;
        }
        else if(s_out < -MAX_STEERING_ANGLE) 
        {
          s_out = -MAX_STEERING_ANGLE;
        }
        //limit motor_speed
        if(motor_speed > MAX_MOTOR_SPEED) 
        {
          motor_speed = MAX_MOTOR_SPEED;
        }
        else if(motor_speed < MIN_MOTOR_SPEED) 
        {
          motor_speed = MIN_MOTOR_SPEED;
        }
        // flatten s_out (Lowpassfilter)
		// Race mode
          s_out_av = weighted_average(s_out, s_out_ar, &s_out_init, RANGE_OF_STEERING_AVG_RACE);
    
    // steering adjustments TODO
    int current_steering = s_out_av;
    last_steer = current_steering;
    steering.data = (int)s_out_av;
    ROS_INFO("State = %d, line = %d, steer = %d, IST = %f, I SEE = %d",current_drive_state, line_selection, s_out_av, istwert, recognized);
    // speed control and collision protection
    if (collision_protection((double)usf.range))
    {
      motor.data = 0;
    }
    else
    {
      motor.data = motor_speed;
    }
    if (stop)
    {
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