//======================================
// RIGHT WHEEL SETUP
//======================================

void right_speed_setup() {
  last_e0 = count_e0; // this is for speed
  speed_e0 = 0.0;
  right_speed_est_ts = millis();
}

float right_smooth_velocity( float speed) {
  // float alpha = 0.1; //smoothing factor, really small alpha introduces delay, but isn't spiky
  return (alpha * speed ) + ((1.0 - alpha)*previous_smooth_speed_right);
}

void right_wheel_count_speed() {
  unsigned long elapsed_time = millis() - right_speed_est_ts;

  if ( elapsed_time >= SPEED_EST_MS) {

    long count_difference =  count_e0 - last_e0;

    last_e0 = count_e0;

    speed_e0 = (float)count_difference / (float)elapsed_time; // this is counts per second
    
    //Serial.println( speed_e0, 4 );
    // Serial.print( 10 );
    // Serial.print(",");
    // Serial.print( 0 );
    // Serial.print(",");
    // Serial.println( smooth_velocity( speed_e0 )*10, 4 ); // pwm of 30 = 0.55 counts per second
    
    right_speed_est_ts = millis(); 
    previous_smooth_speed_right = right_smooth_velocity( speed_e0);
  }
  
  //motors.setPWM(30,30);
}

//======================================
// LEFT WHEEL SETUP
//======================================

void left_speed_setup() {
  last_e1 = count_e1; // this is for speed left
  speed_e1 = 0.0;
  left_speed_est_ts = millis();
}

float left_smooth_velocity( float speed) {
  // float alpha = 0.1; //smoothing factor, really small alpha introduces delay, but isn't spiky
  
  return (alpha * speed ) + ((1.0 - alpha)*previous_smooth_speed_left);
}

void left_wheel_count_speed() {
  unsigned long elapsed_time = millis() - left_speed_est_ts;

  if ( elapsed_time >= SPEED_EST_MS) {

    long count_difference =  count_e1 - last_e1;

    last_e1 = count_e1;

    speed_e1 = (float)count_difference / (float)elapsed_time; // this is counts per second
    
    // //Serial.println( speed_e0, 4 );
    // Serial.print( 10 );
    // Serial.print(",");
    // Serial.print( 0 );
    // Serial.print(",");
    // Serial.println( left_smooth_velocity( speed_e1 )*10, 4 ); // pwm of 30 = 0.55 counts per second
    
    left_speed_est_ts = millis(); 
    previous_smooth_speed_left = left_smooth_velocity( speed_e1);
  }
  
  //motors.setPWM(30,30);
}


//======================================
// FUNCTIONS TO TEST
//======================================

void test_left_speed_pid () {
  left_wheel_count_speed(); // updates left_smooth_velocity
  float measurement_left = left_smooth_velocity( speed_e1);

  if ( millis() - test_ts > TEST_MS ) {
    test_ts = millis();
    demand = demand*(-1.0);
  }

  if (millis() - pid_ts >= PID_MS) {
    float l_pwm = left_pid.update(demand, measurement_left);
    motors.setPWM(l_pwm, 0);
    pid_ts = millis();
  }

  // // Serial prints to help us debug and tune
  Serial.print( 10 );
  Serial.print( "," );
  Serial.print( -10 );
  Serial.print( "," );
  Serial.print( demand*10 );
  // Serial.print( "," );
  // // Serial.print( (demand - measurement)*10 );
  Serial.print( "," );
  Serial.print( measurement_left*10 );

  Serial.print( "\n" );
}

void test_speed_pid () {
  right_wheel_count_speed(); // this does speed measurement every 
  left_wheel_count_speed(); // updates left_smooth_velocity
  float measurement_right = right_smooth_velocity( speed_e0); // this is measured in counts per ms, 0.5 is about 30 pwm
  float measurement_left = left_smooth_velocity( speed_e1);

  if ( millis() - test_ts > TEST_MS ) {
    test_ts = millis();
    demand = demand*(-1.0);
  }

  if (millis() - pid_ts >= PID_MS) {
    
    float r_pwm = right_pid.update(demand, measurement_right);
    float l_pwm = left_pid.update(demand, measurement_left);
    motors.setPWM(l_pwm, r_pwm);
    pid_ts = millis();
  }
  // Serial prints to help us debug and tune
  Serial.print( 10 );
  Serial.print( "," );
  Serial.print( -10 );
  Serial.print( "," );
  Serial.print( demand*10 );
  Serial.print( "," );
  Serial.print( measurement_left*10 );
  Serial.print( "," );
  Serial.print( measurement_right*10 );
  Serial.print( "\n" );
}

//======================================
// ACTUAL FUNCTION TASKS
//======================================

void forwards_constant_speed (float speed_demand) {
  demand = speed_demand;
  right_wheel_count_speed(); // this does speed measurement every 
  left_wheel_count_speed(); // updates left_smooth_velocity
  float measurement_right = right_smooth_velocity( speed_e0); // this is measured in counts per ms, 0.5 is about 30 pwm
  float measurement_left = left_smooth_velocity( speed_e1);

  if (millis() - pid_ts >= PID_MS) {
    
    float r_pwm = right_pid.update(demand, measurement_right);
    float l_pwm = left_pid.update(demand, measurement_left);
    motors.setPWM(l_pwm, r_pwm);
    pid_ts = millis();
  }
  // Serial prints to help us debug and tune
  Serial.print( 10 );
  Serial.print( "," );
  Serial.print( -10 );
  Serial.print( "," );
  Serial.print( demand*10 );
  Serial.print( "," );
  Serial.print( measurement_left*10 );
  Serial.print( "," );
  Serial.print( measurement_right*10 );
  Serial.print( "\n" );
}

void pid_turning_and_forwards(float difference, float speed_demand) {
  demand = speed_demand;
  right_wheel_count_speed(); // this does speed measurement every 
  left_wheel_count_speed(); // updates left_smooth_velocity
  float measurement_right = right_smooth_velocity( speed_e0); // this is measured in counts per ms, 0.5 is about 30 pwm
  float measurement_left = left_smooth_velocity( speed_e1);

  left_wheel_demand = demand + demand*(difference*4); // difference is -1, 1 and -1 is left bumper pressed
  right_wheel_demand = demand + -demand*(difference*4);

//// emily breaking things
//  float left_wheel_demand = demand - demand*(difference*4); // difference is -1, 1 and -1 is left bumper pressed
//  float right_wheel_demand = demand + demand*(difference*4);

  if (millis() - pid_ts >= PID_MS) {
    float l_pwm = left_pid.update( left_wheel_demand, measurement_left);

    float r_pwm = right_pid.update( right_wheel_demand, measurement_right);
    
    motors.setPWM(l_pwm, r_pwm);
    pid_ts = millis();
  }
  Serial.print( 10 );
  Serial.print( "," );
  Serial.print( -10 );
  Serial.print( "," );
  Serial.print( left_wheel_demand*10 );
  Serial.print( "," );
  Serial.print( right_wheel_demand*10 );
  Serial.print( "," );
  Serial.print( bumpers.calibrated_digital[0]*10 );
  Serial.print( "," );
  Serial.print( bumpers.calibrated_digital[1]*10 );
  Serial.print( "," );
  Serial.print( bumpers.left_noise_filter(bumpers.calibrated_digital[0]) *10);
  Serial.print( "," );
  Serial.print( bumpers.right_noise_filter(bumpers.calibrated_digital[1]) *10);
  Serial.print( "\n" );
}
