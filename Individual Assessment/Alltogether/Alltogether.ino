
#include "Motors.h"         // Labsheet 1
#include "PID.h"            // Labsheet 1 - Advanced
#include "LineSensors.h"    // Labsheet 2
#include "Magnetometer.h" // Labsheet 3
#include "Kinematics.h"     // Labsheet 4



#include <Math.h>

#include "Encoders.h"    

#define BUZZER_PIN 6

Motors_c motors;    

LineSensors_c line_sensors; 

Magnetometer_c magnetometer;

Kinematics_c pose; 


struct pair {
  float first, second; 
}; 



unsigned long pose_time_stamp;


///GLOBAL VALUES///

int pid_distance_threshold = 1; // threshold for rotating left and right around centre
int distance_threshold = 4; // this makes object stop at desired location
int distance_threshold_adj = 0; // this makes it target the location withought fucking up spinning. this is for close to location
float velocity_distance_threshold = 0.001; // this corrects movement pid so we dont get nan float numbers
float theta_threshold = 0.3; // was 0.3

float magnet_detection_threshold = 4.0;//was 3???

int normal_speed = 15; // was 20

// Define rectangle coordinates
int locB[] = {260 , -420};
int locA[] = {locB[0] , locB[1] + 180};
int locC[] = {locB[0] - 250 , locB[1]};
int locD[] = {locB[0] - 250 , locB[1] + 180};
//int locE[] = { locA[0] - 70, locA[0] };
//int locF[] = { locA[0] - 70, locB[1] +50};
 

float mag_x, mag_y, mag_theta;


void setup() {
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode( BUZZER_PIN, OUTPUT );


  motors.initialise();

 
  line_sensors.initialiseForADC();

  magnetometer.initialise();

  setupEncoder0();
  setupEncoder1();

  pose.initialise( 0,0,0 );

  // Serial for debug output
  Serial.begin(9600);
  analogWrite( BUZZER_PIN, 120 );
  delay(10);
  analogWrite( BUZZER_PIN, 0 );
  delay(2000);
  Serial.println(" *** READY *** ");

  calibrate_all();

  
  
} // end of setup()

void calibrate_all( ){

 
  while (pose.theta < 2*2*3.14159265) { // rotate 
    
    pose.update();
    centred_left(0,0);
    
    line_sensors.calibrate_line_sensors();
    magnetometer.calibrate_magnetometer();
    
  }
  stop_motors();
  //search_start_time = millis();
}


void stop_motors() {
  motors.setPWM(0,0);
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

////////////////////////////////////////////////
///////////////////////////////////////////////////
//////////FIXING TURNING///////////////
/////////////////////////////

////////////////////////////////////////////////
///////////////////////////////////////////////////
//////////FIXING TRAVELING///////////////
/////////////////////////////



void traveling_pid(float desired_x, float desired_y) {
  pair V_global = pid(desired_x, desired_y);
  float V_global_x = V_global.first;
  float V_global_y = V_global.second;

  // Convert global velocity to robot's local frame
  float cos_theta = cos(pose.theta);
  float sin_theta = sin(pose.theta);
  float V_local_x = V_global_x * cos_theta + V_global_y * sin_theta;
  float V_local_y = -V_global_x * sin_theta + V_global_y * cos_theta;

  float v_normalised = sqrt( V_local_x*V_local_x + V_local_y*V_local_y);
  
  
  if ( v_normalised > velocity_distance_threshold ) {
    v_normalised = sqrt( V_local_x*V_local_x + V_local_y*V_local_y);
  } else { v_normalised = 1; }

  V_local_x = V_local_x*normal_speed / v_normalised;
  V_local_y = V_local_y*normal_speed / v_normalised;
  // Calculate angular adjustment from lateral error
  //float Kp_theta = 1.0; // Tune this value during testing
  float adjustment =  V_local_y; //* Kp_theta ;

  // Calculate PWM values for each wheel
  float left_pwm = V_local_x - adjustment;
  float right_pwm = V_local_x + adjustment;
//  Serial.print( left_pwm, 4 );
//  Serial.print( "," );
//  Serial.print( right_pwm, 4 );
//  Serial.print( "\n" );

  // Constrain PWM values to ±30
  left_pwm = constrain(left_pwm, -normal_speed, normal_speed);
  right_pwm = constrain(right_pwm, -normal_speed, normal_speed);

  motors.setPWM(left_pwm, right_pwm);
}

bool close_to_location( float xp, float yp, int adjuster = 0) {
  float dx = xp - pose.x;
  float dy = yp - pose.y;
  float distance = sqrt(dx * dx + dy * dy);

  return ( distance <= distance_threshold + adjuster);
}

void travel_to( float desired_x, float desired_y) {
  if ( close_to_location ( desired_x, desired_y, distance_threshold_adj) ) {
    stop_motors();
    delay(500);
  } 
  else {
    traveling_pid( desired_x, desired_y);
  }
}
//////////////////////////////////////////////////////////////////////////////
//////////////PUTTING TURNING AND TRAVELING TOGETHER///////////////////////////
///////////////////////////////////////////////////////////////////////////////


void go_to_there_from_here( float desired_x, float desired_y, float from_x, float from_y) {
  float target_theta = angle_from_x_y_to_xp_yp( pose.x, pose.y, desired_x, desired_y);
  
  if ( close_to_location ( from_x, from_y) and !close_to_theta( target_theta) ) {
    turn_to_desired_about( desired_x, desired_y, from_x, from_y);
  } 
  else {
    travel_to(desired_x, desired_y);
  }
}


///////////////////FINDING STUFF////////////////
//////////////////////////////////////////////
void go_home_and_look_to_top_left () {
  static enum { TRAVELING, TURNING} state = TRAVELING;

  switch(state) {
    case TRAVELING:
      travel_to(0, 0);
      if ( pose.x < 2 and pose.y < 2 ) {
        state = TURNING;
      }

      break;
    case TURNING:
      
      //
      float target_theta = angle_from_x_y_to_xp_yp( 0, 0, 200, 150);
      float pose_theta = pose.theta;

      target_theta = mod2pi(target_theta);
      pose_theta = mod2pi(pose_theta);
      
      if ( target_theta < pose_theta ) {
        centred_right(0,0);
        
      } else {
        motors.setPWM(0,0);

      }
      
      break;
  }
  
}

bool magnet_found() { return magnetometer.magnet_found( magnet_detection_threshold );}

// Define a struct to hold the actual magnet position (optional)


// Function to compute the actual magnet location
pair calculate_actual_magnet_position(float mag_x, float mag_y, float mag_theta) {
  pair
  actual;
  const float distance_mm = 50.0; // 5 cm = 50 mm

  // Adjust the position using trigonometry
  actual.first = mag_x + distance_mm * cos(mag_theta);
  actual.second = mag_y + distance_mm * sin(mag_theta);

  return actual;
}

void stop_at_magnet_and_save_location() {
  static enum { IDLE, STOPPING, GOING_HOME, WAITING_AT_HOME, GOING_TO_MAGNET } state = IDLE;
  
  static unsigned long stopStartTime;
  static bool prevMagnetState = false;

  // Check for new magnet detection (rising edge)
  bool currentMagnetState = magnet_found();
  bool magnetTrigger = (currentMagnetState && !prevMagnetState);
  prevMagnetState = currentMagnetState;

  switch(state) {
    case IDLE:
      if(magnetTrigger) {
        // Save location once when magnet is first detected
        mag_x = pose.x;
        mag_y = pose.y;
        mag_theta = pose.theta;
        
        // Begin stop sequence
        motors.setPWM(0, 0);
        stopStartTime = millis();
        state = STOPPING;
      }
      break;

    case STOPPING:
      if(millis() - stopStartTime >= 3000) {
        // After 1 second non-blocking delay, go home
        state = GOING_HOME;
      }
      break;

    case GOING_HOME:
        // Start homing process
        go_home_and_look_to_top_left();

        if ( close_to_location( 0, 0, 5) ){ 
          static unsigned long delay_wait_time = millis();
          if ( millis() - delay_wait_time > 10000) {
            state = GOING_TO_MAGNET;
            
            }
          
          }
        
      break;
    case GOING_TO_MAGNET:
        pair actual = calculate_actual_magnet_position(mag_x, mag_y, mag_theta);
        travel_to(actual.first, actual.second);
      break;
  }
}

void beep_if_magnet_found() {
  magnetometer.calcCalibratedMag();
  
  if ( magnet_found() ) {
    //digitalWrite(LED_BUILTIN, HIGH);
    analogWrite( BUZZER_PIN, 120 );
    stop_motors();
  }
  else {
    //digitalWrite(LED_BUILTIN, LOW);
    analogWrite( BUZZER_PIN, 0 );
  }
}

void turn_if_line_found() {
  if( line_sensors.online() ) { // this loads calibration and detects line
    motors.setPWM(-normal_speed,normal_speed);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    motors.setPWM(0,0);
    digitalWrite(LED_BUILTIN, LOW);
  }
}


 
// MAIN TASKS

void leaving_home_or_looking_for_magnet() {
  static enum { PHASE_LEAVE, PHASE_SEARCH, MAGNET_FOUND } state = PHASE_LEAVE;
  magnetometer.calcCalibratedMag();
  
  switch(state) {
    case PHASE_LEAVE:
   
      go_to_there_from_here(locA[0],locA[1], 0 , 0);
  
      if ( close_to_location(locA[0], locA[1])) {
        state = PHASE_SEARCH;
      }
      break;
    case PHASE_SEARCH:
       static unsigned long search_start_time = millis();
       if ( magnet_found() ) {
        state = MAGNET_FOUND;
      }
      
       if ( millis() - search_start_time > 2*60000) { // go home after 2 min//
        go_home_no_magnet_found();
    } else {
        make_a_rectangle();
    }
      break;
    
    case MAGNET_FOUND:
      stop_at_magnet_and_save_location();
      break;
    
    }
}

void go_home_no_magnet_found(){
  static enum { STOPPING, TRAVELING, TURNING} state = STOPPING;
  static float O[] = {16,-3};

  switch(state) {
    case STOPPING:
      static unsigned long stop_time_stamp = millis();
      motors.setPWM(0,0);
      analogWrite( BUZZER_PIN, 120 );
      delay(500);
      analogWrite( BUZZER_PIN, 0 );
      
      delay(5000);
      
      state = TRAVELING;
      
      break;
    case TRAVELING:
      
      travel_to(O[0],O[1]);
      
      if ( close_to_location( O[0],O[1]) ) {
        state = TURNING;
      }

      break;
    case TURNING:
      
      //
      float target_theta = angle_from_x_y_to_xp_yp( 0, 0, 200, 150);
      float pose_theta = pose.theta;

      target_theta = mod2pi(target_theta);
      pose_theta = mod2pi(pose_theta);
      
      if ( target_theta < pose_theta ) {
        centred_right(O[0],O[1]);
        
      } else {
        motors.setPWM(0,0);
      }
      
      break;
  }
  
}


enum RectangleStates {
  RECT_INIT,
  STAGE1_TO_A,
  STAGE2_TO_B,
  STAGE3_TO_C,
  STAGE4_TO_D,
};

RectangleStates rectState = RECT_INIT;

void make_a_rectangle() {

  
  switch(rectState) {
    case RECT_INIT:
      rectState = STAGE1_TO_A;
      break;

    case STAGE1_TO_A:
      go_to_there_from_here(locA[0], locA[1], locD[0], locD[1]);
      
      if (close_to_location(locA[0], locA[1])) {
        rectState = STAGE2_TO_B;
      }
      
      break;

    case STAGE2_TO_B:
      go_to_there_from_here(locB[0], locB[1], locA[0], locA[1]);
      
      if (close_to_location(locB[0], locB[1])) {
        rectState = STAGE3_TO_C;
      }
      break;

    case STAGE3_TO_C:
      go_to_there_from_here(locC[0], locC[1], locB[0], locB[1]);
      
      if (close_to_location(locC[0], locC[1])) {
        rectState = STAGE4_TO_D;
      }
      break;

    case STAGE4_TO_D:
      go_to_there_from_here(locD[0], locD[1], locC[0], locC[1]);

      if ( close_to_location(locD[0], locD[1]) ) {
        rectState = STAGE1_TO_A;
      }
      
      break;
  }
}


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

void loop() {
  static enum { IDLE, MAGNET_FOUND } state = IDLE;
  magnetometer.calcCalibratedMag();
  
  leaving_home_or_looking_for_magnet();
      

  
  
  

  unsigned long elapsed_pose_time = millis() - pose_time_stamp;
  
  if ( elapsed_pose_time > 20) { //update position every 20ms.
    
    pose.update();
    pose_time_stamp = millis();
  }



} 
