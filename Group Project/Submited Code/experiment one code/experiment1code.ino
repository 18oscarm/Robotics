
#include "Motors.h"         // Labsheet 1
#include "PID.h"            // Labsheet 1 - Advanced
#include "LineSensors.h"    // Labsheet 2
#include "Kinematics.h"     // Labsheet 4

#include "Encoders.h"     // For encoder counts

#include <Math.h>

#define BUZZER_PIN 6

//===============
//ODOMETRY DEFINITIONS
//===================

#define MAX_RESULTS 105
#define VARIABLES 5
#define STATE_RUNNING_EXPERIMENT 0
#define STATE_FINISHED_EXPERIMENT 1
#define EXPERIMENT_END_MS 15000 // 5 sec



Motors_c motors;    

BumpSensors_c bumpers;

Kinematics_c pose; 

PID_c right_pid;

PID_c left_pid;

////////////////PID demand and test
float demand = 0.0; // demand wheel speed

unsigned long test_ts;
# define TEST_MS 2000

float left_wheel_demand;
float right_wheel_demand;

// PID Scheduling
unsigned long pid_ts;
#define PID_MS 50 // Update PID every 50ms

//////////////// SPEED measurement
unsigned long right_speed_est_ts;
unsigned long left_speed_est_ts;
#define SPEED_EST_MS 10 //estimate speed every 10 ms
long last_e0 = 0;
float speed_e0 = 0;
float previous_smooth_speed_right = 1.0;

long last_e1 = 0;
float speed_e1 = 0;
float previous_smooth_speed_left = 1.0;


float alpha = 0.09;// smothening constant, a low alpha gives smoother responses, but with a bit of lag
float Kp = 30.0; // this basically gives an initial jolt to get the robot moving. this value is so high because 30 pwm is about 0.5 "speed"
float Ki = 0.1; // this pulles the pwm the rest of the way to the desired speed. this is cumulative, so don't make too high
float Kd = 0.0; // no clue what this does, not in labsheet


//============================
//GLOBAL ODOMETRY VARIABLES
//=============================

short results[MAX_RESULTS] [VARIABLES];
int results_index = 0;
int state;
unsigned long experiment_start_ts;
unsigned long record_results_ts;
unsigned long results_interval_ms;



///GLOBAL VALUES///
struct pair {
  float first, second; 
}; 



unsigned long pose_time_stamp;

int pid_distance_threshold = 1; // threshold for rotating left and right around centre
int distance_threshold = 4; // this makes object stop at desired location
int distance_threshold_adj = 0; // this makes it target the location withought fucking up spinning. this is for close to location
float velocity_distance_threshold = 0.001; // this corrects movement pid so we dont get nan float numbers
float theta_threshold = 0.3; // was 0.3

float magnet_detection_threshold = 4.0;//was 3???

int normal_speed = 30; // was 

void setup() {


  pinMode(LED_BUILTIN, OUTPUT);
  pinMode( BUZZER_PIN, OUTPUT );


  motors.initialise();
  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);

  results_interval_ms = (EXPERIMENT_END_MS / MAX_RESULTS);
  state = STATE_RUNNING_EXPERIMENT;

  Serial.begin(9600);
  analogWrite( BUZZER_PIN, 120 ); // on
  delay(100);
  analogWrite( BUZZER_PIN, 0 ); // off
  delay(2000);
  Serial.println(" *** READY *** ");

  calibrate_all();

  right_pid.initialise( Kp, Ki, Kd); // kp of 50 is ok for no dampening, 300 is good for with dampening
  left_pid.initialise( Kp , Ki, Kd);

  test_ts = millis();

  right_pid.reset();
  left_pid.reset();

  right_speed_setup();
  left_speed_setup();
  
  experiment_start_ts = millis(); //for odometry experiment
  record_results_ts = millis(); 
}

void calibrate_all(){
  unsigned long start = millis();
  unsigned long current = start;
  while ( (current - start) < 5000) {
      //motors.setPWM(30,-30);
      bumpers.calibrateDigitalBumpers();
      current = millis();
  }
  motors.setPWM(0,0);
  analogWrite( BUZZER_PIN, 120 ); // on
  delay(100);
  analogWrite( BUZZER_PIN, 0 ); // off
}


void turn_to_after_calibrating (float target_theta) {
  unsigned long start = millis();
  unsigned long current = start;
  while ( (current - start) < 3000) {
    turn_to_desired_about(100,100,0,0);
  }
  motors.setPWM(0,0);
  analogWrite( BUZZER_PIN, 120 ); // on
  delay(100);
  analogWrite( BUZZER_PIN, 0 ); // off
}

pair pid(float desired_x, float desired_y){ 
  // Calculate position error
  float dx = desired_x - pose.x;
  float dy = desired_y - pose.y;

  // Calculate direction towards initial position
  float distance = sqrt(dx*dx + dy*dy);
  float V_global_x = 0.0;
  float V_global_y = 0.0;

  if (distance > pid_distance_threshold) { // Avoid division by zero
    V_global_x = dx / distance; // normalised distances
    V_global_y = dy / distance;
  }

  // Proportional gain (adjust based on testing)
  float Kp = 1.0; 
  V_global_x *= Kp * distance; 
  V_global_y *= Kp * distance; 

  return {V_global_x, V_global_y};
}

int turn_pid(float about_x, float about_y) { // this is for turning about x and y
  
  pair V_global = pid( about_x, about_y);
  float V_global_x = V_global.first;
  float V_global_y = V_global.second;

  // Transform global corrective velocity to robot's frame
  float V_corrective = V_global_x * cos(pose.theta) + V_global_y * sin(pose.theta);

  int adjust = (int)V_corrective;

  return adjust;
}


void centred_right(float about_x, float about_y) { // this allows me to turn fixed about the desired location
  int base_pwm = normal_speed;
  int adjust = turn_pid( about_x,  about_y);

  // Apply correction to both motors
  int left_pwm = base_pwm + adjust;
  int right_pwm = -base_pwm + adjust;

  // Clamp PWM values to valid range (-255 to 255)
  left_pwm = constrain(left_pwm, -normal_speed, normal_speed);
  right_pwm = constrain(right_pwm, -normal_speed, normal_speed);

  motors.setPWM(left_pwm, right_pwm);
}

void centred_left(float about_x, float about_y) {
  int base_pwm = normal_speed;
  int adjust = turn_pid( about_x,  about_y);

  // Apply correction to both motors
  int left_pwm = -base_pwm + adjust;
  int right_pwm = base_pwm + adjust;

  // Clamp PWM values to valid range (-255 to 255)
  left_pwm = constrain(left_pwm, -normal_speed, normal_speed);
  right_pwm = constrain(right_pwm, -normal_speed, normal_speed);

  motors.setPWM(left_pwm, right_pwm);
}


float mod2pi(float value) { // returns a value between 0 and 2pi
    float two_pi = 2.0 * 3.14159265;
    float result = fmod(value, two_pi);
    if (result < 0) {
        result = result + two_pi;  // Ensure the result is always positive
    }
    return result;
}

float angle_from_x_y_to_xp_yp(float x, float y, float x_prime, float y_prime) {
    return atan2(y_prime - y, x_prime - x);  // Returns angle to turn to
}


void setTurn( float desired_x, float desired_y, float about_x, float about_y) { // spin fixed about x and y
  float target_theta = angle_from_x_y_to_xp_yp( pose.x, pose.y, desired_x, desired_y);
  
  float pi = 3.14159265;
  
  target_theta = mod2pi( target_theta); 
  
  float pose_theta = mod2pi( pose.theta); // current angle position mod 2*pi
  
  if (target_theta > pose_theta ) {
    float delta_theta = target_theta - pose_theta;
    if (delta_theta < pi) {
      //turn left
      centred_left(  about_x, about_y);
      //normal_left(); // centred instead of normal left
    }
    else {
      //turn right
      centred_right( about_x, about_y);
      //normal_right();
    }
  }
  else{
    float delta_theta = pose_theta - target_theta;
    if (delta_theta < pi) {
      //turn right
      centred_right( about_x, about_y);
      //normal_right();
    }
    else{
     //turn left /
     centred_left( about_x, about_y);
     //normal_left();
    }
  }

}


bool close_to_theta( float target_theta) { // checks if theta_target is close to pose.theta  
  target_theta = mod2pi( target_theta); 
  float pose_theta = mod2pi( pose.theta ); 
  float delta_theta = (target_theta - pose_theta);

  if ( abs( delta_theta) <= theta_threshold  ) { 
    return true; // not turning because we are close enough
  }
  else { 
    return false;
    }
}

void turn_to_desired_about( float desired_x, float desired_y, float about_x, float about_y){ // we turn on the spot about xy to face desired xy
  float target_theta = angle_from_x_y_to_xp_yp( pose.x, pose.y, desired_x, desired_y);

  if ( close_to_theta( target_theta) ) {
    //motors.setPWM(0,0);    
    
  }
  else {
    // i want to initialise and fix about x and y
    setTurn( desired_x, desired_y, about_x, about_y );
  }
}

// put your main code here, to run repeatedly:
void loop() {
  float difference = bumpers.bumper_difference(); // between 0 and 1, left is 0 and right is one
  pid_turning_and_forwards(difference, 0.2);
  //forwards_constant_speed(0.2);
  odometry();
}
