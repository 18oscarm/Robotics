/***************************************
 ,        .       .           .     ,-.  
 |        |       |           |        ) 
 |    ,-: |-. ,-. |-. ,-. ,-. |-      /  
 |    | | | | `-. | | |-' |-' |      /   
 `--' `-` `-' `-' ' ' `-' `-' `-'   '--' 
****************************************/

#ifndef _LINESENSORS_H
#define _LINESENSORS_H

// Bumper sensor configuration
#define EMIT_PIN 11
#define NUM_BUMPERS 2

#define DIGITAL_TIMEOUT_US 4000

// Digital pins for bumpers (left=4, right=5)
const int bumper_pins[NUM_BUMPERS] = {4, 5};



class BumpSensors_c {
  public:
    




    // Digital reading storage
    unsigned long digital_readings[NUM_BUMPERS];
    
    // Calibration data
    unsigned long min_digital[NUM_BUMPERS] = {1024, 1024};
    unsigned long max_digital[NUM_BUMPERS] = {0, 0};
    float calibrated_digital[NUM_BUMPERS];

    BumpSensors_c() {}



    // ==============================================
    // Digital Methods (Both bumpers)
    // ==============================================

    void initialiseForDigital() {
      pinMode(EMIT_PIN, OUTPUT);
      digitalWrite(EMIT_PIN, HIGH); // maybe set to low later, it didnt work as high before....
    }

    unsigned long digitalReadBumper(int bumper_index) {
      if(bumper_index < 0 || bumper_index >= NUM_BUMPERS) return 0;
      
      int pin = bumper_pins[bumper_index];
      
      // Charge capacitor and enable IR LED
      pinMode(pin, OUTPUT);
      digitalWrite(pin, HIGH);
      delayMicroseconds(10);
      pinMode(pin, INPUT);

      // Measure discharge time
      unsigned long start = micros();
      while (digitalRead(pin) == HIGH) {
        if(micros() - start > DIGITAL_TIMEOUT_US) break;
      }
      return micros() - start;
    }

    void readDigitalBumpers() {
      initialiseForDigital();
      for(int i = 0; i < NUM_BUMPERS; i++) {
        digital_readings[i] = digitalReadBumper(i);
      }
      pinMode(EMIT_PIN, INPUT);  // Turn off IR LEDs
    }

    // ==============================================
    // Calibration Methods
    // ==============================================
    void calibrateDigitalBumpers() {
      readDigitalBumpers();
      for(int i = 0; i < NUM_BUMPERS; i++) {
        if(digital_readings[i] < min_digital[i]) {
          min_digital[i] = digital_readings[i];
        }
        if(digital_readings[i] > max_digital[i]) {
          max_digital[i] = digital_readings[i];
        }
      }
    }

    void calcCalibratedDigital() {
      readDigitalBumpers();
      for(int i = 0; i < NUM_BUMPERS; i++) {
        unsigned long range = max_digital[i] - min_digital[i];
        if (range <=0) {
          calibrated_digital[i] = 0.0;
        } else {
          calibrated_digital[i] = 1.0 - ( ((float)digital_readings[i] - (float)min_digital[i]) / (float)range);
        
          
        }
      
      }
      
    }
    float left_noise_filter( float bumper) {
    static float left_last_bumper = 0;
    // float alpha = 0.1; //smoothing factor, really small alpha introduces delay, but isn't spiky
    float beta = 0.1;
    left_last_bumper = (beta * bumper ) + ((1.0 - beta)*left_last_bumper);
    return left_last_bumper;
}   
    float right_noise_filter( float bumper) {
      static float right_last_bumper = 0;

      float beta = 0.1;
      right_last_bumper = (beta * bumper ) + ((1.0 - beta)*right_last_bumper);
      return right_last_bumper;
    }

    float bumper_difference() { 
      float bumper_threshold = 0.1;
      calcCalibratedDigital();
      float left_bumper = left_noise_filter(calibrated_digital[0]);
      float right_bumper = right_noise_filter(calibrated_digital[1]);
      float difference = (right_bumper - left_bumper); //-1 means turn left and +1 means turn right, avg is 1?
      if (bumper_threshold > abs(difference)) {
        difference = 0;
      }
      return difference;
    }
};

#endif
