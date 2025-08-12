


///GLOBAL VALUES FOR ODOMETRY STORAGE ///




//==============================
// ENCODING
//==============================

short distance_storage_encoding(float distance) { //stores int values from -32768 to 32767 (32 meters). we dont care about decimal places
  short trunc_distance = (short)distance;
  return trunc_distance;
}

short angle_storage_encoding(float angle) { //angle in radians
  float scaled_angle = angle*100;  //0.01 rad = 0.5 degrees, so we can still preserve 2 d.p.
  short trunc_angle = (short)scaled_angle;
  return trunc_angle;
}

short bumper_storage_encoding(float bump) { // we are only storing values between 0 and 1, so we can scale it all up to 4sf
  bump = bump*10000;
  short trunc_bump = (short)bump;
  return trunc_bump;
}

//=======================================
// REVERTING
// ====================================

float revert_angle_to_float (short angle) {
  float reverted_angle = (float)angle;
  reverted_angle = reverted_angle/100;
  return reverted_angle;
}


float revert_bump_to_float(short scaled_bump) {
  float float_scaled_bump = (float)scaled_bump;
  float bump = float_scaled_bump/10000;
  return bump;
}

//============================
//ODOMETRY
//============================

void running_experiment() {
  //motors.setPWM (20,20);
  bumpers.calcCalibratedDigital();
  pose.update();

  unsigned long total_exp_time = millis() - experiment_start_ts;
  unsigned long elapsed_time = millis() - record_results_ts;

  if (total_exp_time > EXPERIMENT_END_MS) {
    
    analogWrite( BUZZER_PIN, 120 );
    delay(10);
    analogWrite( BUZZER_PIN, 0 );
    Serial.println("Switching to FINISHED state"); // Debug
    state = STATE_FINISHED_EXPERIMENT;
    //return;
    
  } else {
    

    if( elapsed_time > results_interval_ms ) { //saves all data
      record_results_ts = millis();

      if(results_index < MAX_RESULTS) {
            results[ results_index ][0] = distance_storage_encoding(pose.x); ; // save x (from kinematics?)
            results[ results_index ][1] = distance_storage_encoding(pose.y); // save y (from kinematics?)
            results[ results_index ][2] = angle_storage_encoding(pose.theta); // save theta (from kinematics?)
            results[ results_index ][3] = bumper_storage_encoding(bumpers.calibrated_digital[0]); // save left? bumper
            results[ results_index ][4] = bumper_storage_encoding(bumpers.calibrated_digital[1]); // save right? bumper
            results_index++;
            
      } else {
        state = STATE_FINISHED_EXPERIMENT;
        return;
      }
    
    }
    
  }
}


void experiment_finished() {
  motors.setPWM (0,0);
  Serial.println("In FINISHED state"); // Debug

  int result;
  Serial.print( "The angle it turned to face the box was");
  Serial.print(",");
  Serial.println(final_theta);
  Serial.println("Sample, X, Y, Theta, LeftB, RightB\n");
  
  for (result = 0; result < MAX_RESULTS; result++) {
  Serial.print( result );
        Serial.print(",");
        Serial.print( results[ result ][0] ); // X
        Serial.print(",");
        Serial.print( results[ result ][1] ); // Y
        Serial.print(",");
        Serial.print( revert_angle_to_float(results[ result ][2]) ); // Theta
        Serial.print(",");
        Serial.print( revert_bump_to_float(results[ result ][3]) ); // left bumper
        Serial.print(",");
        Serial.print( revert_bump_to_float(results[ result ][4]) ); // right bumper
        Serial.print("\n");
  }
  delay(5000);
}


void odometry() {
  if (state == STATE_RUNNING_EXPERIMENT) {
    running_experiment();
  } else if (state == STATE_FINISHED_EXPERIMENT) {
    experiment_finished();
  }
}
