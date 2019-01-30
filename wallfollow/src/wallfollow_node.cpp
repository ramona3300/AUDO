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
// driving states
#define DS_CURVE 1 // curve mode
#define DS_STRAIGHT 2 // straight mode
#define DS_CURVE_AP 3 // curve approach
#define DS_STRAIGHT_AP 4 // straight approach
#define DS_STARTUP 5 // startup mode (straight mode slow)
#define DS_CURVE_SLOW 6 // curve mode slow
#define DS_STRAIGHT_SLOW 7 // straight mode slow
#define DS_CURVE_AP_SLOW 8 // curve approach slow
#define DS_STRAIGHT_AP_SLOW 9 // straight approach slow
#define DS_SWITCH_L2R 10 // lane switch from left to right
#define DS_SWITCH_R2L 11 // lane switch from right to left
#define DS_SWITCH_HARD 12 // high p-value after lane switching 

#define OBSTACLE_ON_RIGHT_LANE 20
#define OBSTACLE_ON_LEFT_LANE 21
#define NO_OBSTACLE 9999

#define RANGE_OF_STEERING_AVG 20 // for s_out lowpassfilter

double last_t, t;
std_msgs::Int16 motor, steering;
bool stop = false;
double x_right;
double x_left;
double x_right_2;
double x_left_2;
double x_right_3;
double x_left_3;
int x_obstacle_r = NO_OBSTACLE;
int x_obstacle_l = NO_OBSTACLE;


double usf_history[RANGE_OF_AVERAGE];
bool usf_flag = true;

/**
	lines_recognized
	Checks which lines have been successfully recognized
	return	1	Both
			2	Only Right 
			3 	Only Left	
**/
int lines_recognized(int line_selection){

	if(	x_left > x_right - 5 ||
		x_left + 5 > x_right){
		// Only One line has been recognized
		// The recognized line is the one that is on the side that we drive
		if(line_selection){
			return 3;
		}else{
			return 2;
		}
	}
	return 1;
}

int obstacle_detection(int line_selection){
    // driving on left lane
    if(line_selection){
        if(    x_left_2 < x_obstacle_r 
            && x_left_2 + 20 > x_obstacle_l)
            {
                return OBSTACLE_ON_LEFT_LANE;
            }
    } 
    // driving on right lane
    if(!line_selection){
        if(    x_right_2 > x_obstacle_l 
            && x_right_2 - 20 < x_obstacle_r)
            {
                return OBSTACLE_ON_RIGHT_LANE;
            }
    } 
    else return NO_OBSTACLE;
    //ROS_INFO("x_obst = %d",x_obstacle);
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

    if(average < 0.35) return true;
    else return false;
}

int drive_state(int line_selection, int *curved, bool *actual_curve, int *straight_delay, int *curve_delay, int drive_mode){
    bool is_curve = false;
    int curve_count = 0;
    double offset = 3;
    double diff = 0;
    double diff_2 = 0;
    // Curve or Straight?
    if(line_selection){// left
        diff = x_left_2 - x_left;
        diff_2 = x_left_3 - x_left_2;
        if( (diff_2 < diff + offset) && 
            (diff_2 > diff - offset)){
            is_curve = false;// no curve
        }else{// curve
            is_curve = true;
        }
    }else{// right
        diff = x_right - x_right_2;
        diff_2 = x_right_2 - x_right_3;
        if( (diff_2 < diff + offset) && 
            (diff_2 > diff - offset)){
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

    //ROS_INFO("count = %d curve = %d curve_del = %d straight_del = %d, diff = %f, diff_2 = %f",curve_count,*actual_curve,*curve_delay,*straight_delay, diff, diff_2);
    //ROS_INFO("x_right = %f x_left = %f x_right_2 = %f x_left_2 = %f x_right_3 = %f x_left_3 = %f",x_right, x_left, x_right_2, x_left_2, x_right_3, x_left_3);
    // when approaching a curve wait for some time before switching to curve mode
    if(curve_count >= 16){
        // approaching a curve
        if(!(*actual_curve)){
            // delay actually being in the curve
            if(*straight_delay == 0){
                *actual_curve = true;
                *curve_delay = 20;
                return drive_mode ? DS_CURVE : DS_CURVE_SLOW;
            }else{
                (*straight_delay)--;
                return drive_mode ? DS_CURVE_AP : DS_CURVE_AP_SLOW;
            }
        }else{
            *actual_curve = true;
            *curve_delay = 20;
            return drive_mode ? DS_CURVE : DS_CURVE_SLOW;
        }
    }else if(curve_count <= 15){
        // approaching a straight
        if(*actual_curve){
            // delay actually being in the straight
            if(*curve_delay == 0){
                *actual_curve = false;
                *straight_delay = 20;
                return drive_mode ? DS_STRAIGHT : DS_STRAIGHT_SLOW;
            }else{
                (*curve_delay)--;
                return drive_mode ? DS_STRAIGHT_AP : DS_STRAIGHT_AP_SLOW;
            }
        }else{
            *actual_curve = false;
            *straight_delay = 20;
            return drive_mode ? DS_STRAIGHT : DS_STRAIGHT_SLOW;
        }
    }
}

int steering_characteristic(int s_out_av, int last_steer){
    // do clever things
    return s_out_av;
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
// receive lane_switch
void lane_switch_Callback(std_msgs::Int32::ConstPtr msg, int* data)
{
  *data = (int) msg->data;
}

// receive x position of obstacle
void obstacle_Callback_right(std_msgs::Int32::ConstPtr msg, int* data)
{
  *data = (int) msg->data;
}
// receive x position of obstacle
void obstacle_Callback_left(std_msgs::Int32::ConstPtr msg, int* data)
{
  *data = (int) msg->data;
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

  // subscribe to ultra sonic
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
      "/line_recoqnition/right_2", 1, boost::bind(right_Callback_2, _1, &x_right_2));
  ros::Subscriber left_sub_2 = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/left_2", 1, boost::bind(left_Callback_2, _1, &x_left_2));
  ros::Subscriber right_sub_3 = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/right_3", 1, boost::bind(right_Callback_3, _1, &x_right_3));
  ros::Subscriber left_sub_3 = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/left_3", 1, boost::bind(left_Callback_3, _1, &x_left_3));
  ros::Subscriber obstacle_sub_right = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/obstacle/right", 1, boost::bind(obstacle_Callback_right, _1, &x_obstacle_r));
  ros::Subscriber obstacle_sub_left = nh.subscribe<std_msgs::Int32>(
      "/line_recoqnition/obstacle/left", 1, boost::bind(obstacle_Callback_left, _1, &x_obstacle_l));

  // switch the lane?
  int lane_switch = 0;
  // subscribe to manual lane switch
  ros::Subscriber lane_switch_sub = nh.subscribe<std_msgs::Int32>(
      "/lane_switch", 1, boost::bind(lane_switch_Callback, _1, &lane_switch));
  
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
  double sollwert_race = 1.8;
  double sollwert_obstacle = 1.4;
  int s_out = 0;
  last_t = ((double)clock()/CLOCKS_PER_SEC);
  int s_out_ar[RANGE_OF_STEERING_AVG];
  for(int i = 0; i < RANGE_OF_STEERING_AVG; i++){s_out_ar[i] = 0;}
  // drive_state variables
  int current_drive_state = DS_STARTUP;
  int current_obstacle_state = NO_OBSTACLE;
  int curved[30];
  for(int i = 0; i < 30; i++){curved[i] = 0;}
  bool actual_curve = false;
  int straight_delay = 0;
  int curve_delay = 0;
  int startup_delay = 20;
  int s_out_av = 0;

  // Select drive mode: 1 = Race mode , 0 = Obstacle Detection mode
  int drive_mode = 0;
  int switch_state_del_r2l = 0;
  int switch_state_del_l2r = 0;
  int switch_ar[10];
  int switch_duration = 30;
  
  if(drive_mode){// Race mode
	sollwert = sollwert_race;
	}else{// Obstacle Detection mode
	sollwert = sollwert_obstacle;
  }

  // #####################################################################################################
  // ##### Loop starts here:    loop rate value is set in Hz                                ##############
  // #####################################################################################################
  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    t = ((double)clock()/CLOCKS_PER_SEC);
    s_out = 0;
    motor_speed = 600; // works with 300 
    

    // which line to follow
    if(line_selection){
        istwert = ( (double)x_left + 65.0 ) / 100.0;// left
    }else{
        istwert = (double)x_right / 100.0;// right
    }
    // #####################################################################################################
    // ##### Controlling                                                                      ##############
    // #####################################################################################################
    // get drive state and obstacle state
	if( 	current_drive_state != DS_SWITCH_R2L
		&&  current_drive_state != DS_SWITCH_L2R
		&&  current_drive_state != DS_SWITCH_HARD){
    current_drive_state = drive_state(line_selection, curved, &actual_curve, &straight_delay, &curve_delay, drive_mode);
	}
    if(drive_mode == 0) current_obstacle_state = obstacle_detection(line_selection);

    // ignore drive_state for some time after start
    if(startup_delay > 0){
        current_drive_state = DS_STARTUP;
        startup_delay--;
    }

    //ignore drive_state if obstacle is detected
    if (current_obstacle_state == OBSTACLE_ON_RIGHT_LANE && !line_selection){
        // car and obstacle are on right lane
        switch_state_del_r2l = switch_duration;
        //line_selection = 1;
    }
    if (current_obstacle_state == OBSTACLE_ON_LEFT_LANE && line_selection){
        // car and obstacle are on left lane
        switch_state_del_l2r = switch_duration;
        //line_selection = 0;
    }
    // delay -- the lane switch takes some time
	int recognized = lines_recognized(line_selection);	
    if(switch_state_del_r2l > 0){
		switch_state_del_r2l--;
		if( 2 == recognized || switch_state_del_r2l > switch_duration - 7){
			// left line is not recognized yet - turn left
			current_drive_state = DS_SWITCH_R2L;
		}else if( 2 != recognized ){
			// left line is recognized
			line_selection = 1; 
			istwert = ( (double)x_left + 65.0 ) / 100.0;// left
			sollwert = 1.4;
			current_drive_state = DS_SWITCH_HARD;
		}	
		if(switch_state_del_r2l < 1){
			sollwert = sollwert_obstacle;
			current_drive_state = DS_STARTUP;
		}
		switch_state_del_l2r = 0;
    }
    if(switch_state_del_l2r > 0){
        switch_state_del_l2r--;
		if( 3 == recognized || switch_state_del_l2r > switch_duration - 7){
			// right line is not recognized yet - turn left
			current_drive_state = DS_SWITCH_L2R;
		}else if( 3 != recognized ){
			// right line is recognized 
			line_selection = 0;
			istwert = (double)x_right / 100.0;// right
			sollwert = 1.4;
			current_drive_state = DS_SWITCH_HARD;
		}
		if(switch_state_del_l2r < 1){
			sollwert = sollwert_obstacle;
			current_drive_state = DS_STARTUP;
		}
        switch_state_del_r2l = 0;
    }

    
    switch(current_drive_state){
        // ####################### Race Mode ###############################
        case DS_CURVE: // Curve mode        1
            motor_speed = (int) ( (double)motor_speed * 0.65 );
            pk = (int) (3000.0 * 0.7 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.22 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_CURVE_AP: // Straight mode  2
            motor_speed = (int) ( (double)motor_speed * 0.65 );
            pk = (int) (3000.0 * 0.4 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.1 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_STRAIGHT: // Straight mode  3
            motor_speed = (int) ( (double)motor_speed * 1.2 );
            pk = (int) (3000.0 * 0.25 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0 / (double)motor_speed ) );
            break;
        case DS_STRAIGHT_AP: // Curve mode  4
            motor_speed = (int) ( (double)motor_speed * 1 );
            pk = (int) (3000.0 * 0.35 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
            break;
        // ####################### Obstacle Detection Mode ###############################
        case DS_CURVE_SLOW: // Curve mode        6
            motor_speed = 300;
            pk = (int) (3000.0 * 0.7 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.1 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_CURVE_AP_SLOW: // Straight mode  7
            motor_speed = 300;
            pk = (int) (3000.0 * 0.15 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_STRAIGHT_SLOW: // Straight mode  8
            motor_speed = 300;
            pk = (int) (3000.0 * 0.15 * ( 300.0 / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0 / (double)motor_speed ) );
            break;
        case DS_STRAIGHT_AP_SLOW: // Curve mode  9
            motor_speed = 300;
            pk = (int) (3000.0 * 0.7 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.1 * ( 300.0  / (double)motor_speed ) );
            break;
        case DS_SWITCH_R2L: // switch from right lane to left lane 11
            motor_speed = 260;
	    break;
        case DS_SWITCH_L2R: //switch from left lane to right lane 10
            motor_speed = 260;
            break;
		case DS_SWITCH_HARD: // high p-value after lane switching  12
	    motor_speed = 270;
            pk = (int) (3000.0 * 1.0 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.1 * ( 300.0  / (double)motor_speed ) );
            break;
        // ####################### Startup Mode ###############################
        case DS_STARTUP:// Straight mode slow  5
            motor_speed = 300;
            pk = (int) (3000.0 * 0.15 * ( 300.0  / (double)motor_speed ) );
            pd = (int) (1000.0 * 0.05 * ( 300.0  / (double)motor_speed ) );
            break;
    }
    
    // overwrite controlling to switch lane
    // mainly to wait until the new line is in the camera focus
    // else calculate controller values
    switch(current_drive_state){
        case DS_SWITCH_R2L:
            s_out_av = -MAX_STEERING_ANGLE;
			for(int i = 0; i < 10; i++){switch_ar[i] = -MAX_STEERING_ANGLE;}
            break;
        case DS_SWITCH_L2R:   
            s_out_av = MAX_STEERING_ANGLE;
			for(int i = 0; i < 10; i++){switch_ar[i] = MAX_STEERING_ANGLE;}
            break;
		case DS_SWITCH_HARD: // no lowpass filter while switching
			// calculate controller values
            err = sollwert - istwert;
            p_err = pk * err;// P-Anteil
            d_err = pd * (err - last_err) / (t - last_t);// D-Anteil
            s_out = -(p_err +  d_err);
            // limit s_out to +-MAX_STEERING_ANGLE
            if(s_out > MAX_STEERING_ANGLE) s_out = MAX_STEERING_ANGLE;
            else if(s_out < -MAX_STEERING_ANGLE) s_out = -MAX_STEERING_ANGLE;
			
			// flatten s_out (Lowpassfilter)
            int temp2[10];
            for(int i = 0; i < 10; i++){temp2[i] = switch_ar[i];}
            for(int i = 1; i < 10; i++){switch_ar[i] = temp2[i - 1];}
            switch_ar[0] = s_out;
            s_out_av = 0;
            for(int i = 0; i < 10; i++){
                s_out_av += switch_ar[i] * 0.1 * (10 - i);
            }
            s_out_av = s_out_av / 5;
            
			
			break;
        default:
            // calculate controller values
            err = sollwert - istwert;
            p_err = pk * err;// P-Anteil
            d_err = pd * (err - last_err) / (t - last_t);// D-Anteil
            s_out = -(p_err +  d_err);
            // limit s_out to +-MAX_STEERING_ANGLE
            if(s_out > MAX_STEERING_ANGLE) s_out = MAX_STEERING_ANGLE;
            else if(s_out < -MAX_STEERING_ANGLE) s_out = -MAX_STEERING_ANGLE;

            // flatten s_out (Lowpassfilter)
            int temp[RANGE_OF_STEERING_AVG];
            for(int i = 0; i < RANGE_OF_STEERING_AVG; i++){temp[i] = s_out_ar[i];}
            for(int i = 1; i < RANGE_OF_STEERING_AVG; i++){s_out_ar[i] = temp[i - 1];}
            s_out_ar[0] = s_out;
            s_out_av = 0;
            for(int i = 0; i < RANGE_OF_STEERING_AVG; i++){
                s_out_av += s_out_ar[i] * 0.1 * (RANGE_OF_STEERING_AVG - i);
            }
            s_out_av = s_out_av / 20;
            break;
    }
    // steering adjustments TODO
    int current_steering = steering_characteristic(s_out_av, last_steer);
    last_steer = current_steering;
    steering.data = (int)s_out_av;
    ROS_INFO("State = %d, line = %d, steer = %d, IST = %f, I SEE = %d",current_drive_state, line_selection, s_out_av, istwert, recognized);
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
