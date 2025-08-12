/***************************************
 ,        .       .           .     ,--, 
 |        |       |           |       /  
 |    ,-: |-. ,-. |-. ,-. ,-. |-     `.  
 |    | | | | `-. | | |-' |-' |        ) 
 `--' `-` `-' `-' ' ' `-' `-' `-'   `-'  
***************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MAGNETOMETER_H
#define _MAGNETOMETER_H

#include <Wire.h>
#include <LIS3MDL.h>

#define MAX_AXIS 3

class Magnetometer_c {

  public:

    // Instance of the LIS3MDL class used to
    // interact with the magnetometer device.
    LIS3MDL mag;

    // A place to store the latest readings 
    // from the magnetometer
    float readings[ MAX_AXIS ];
    float minimum[ MAX_AXIS ];
    float maximum[ MAX_AXIS ];
    
    float scaling[ MAX_AXIS ];
    float offset[ MAX_AXIS ];
    float range[ MAX_AXIS ];
    float calibrated[ MAX_AXIS ];
    
    // Constructor, must exist.
    Magnetometer_c () {
      // Leave this empty.
      // If you put Wire.begin() into this function
      // it will crash your microcontroller.
    }

    // Call this function witin your setup() function
    // to initialise the I2C protocol and the
    // magnetometer sensor
    bool initialise() {
      for ( int n = 0; n < MAX_AXIS ; n++ ) {
        maximum[n] = -9999.9;
        minimum[n] = 9999.9;
      }
      // Start the I2C protocol
      Wire.begin();

      // Try to connect to the magnetometer
      if ( !mag.init() ) {
        return false;
      } else {
        mag.enableDefault();
        return true;
      }
    } // End of initialise()

    // Function to update readings array with
    // latest values from the sensor over i2c
    void getReadings() {
      
      mag.read();
      readings[0] = mag.m.x;
      readings[1] = mag.m.y;
      readings[2] = mag.m.z;
    } // End of getReadings() ??? doesn't work!!!

    void calibrate_magnetometer ( )  {//unsigned long calibration_length ){
      

//      unsigned long start_time = millis();
//      unsigned long end_time = start_time + calibration_length;
      
      //while (millis() < end_time) {
      getReadings();

      for ( int n = 0; n < MAX_AXIS ; n++ ) {
        if ( readings[n] > maximum[n]) {
          maximum[n] = readings[n];
        }

        if ( readings[n] < minimum[n] ) {
          minimum[n] = readings[n];
        }
      }

      //delay(10);
      //}

      for ( int n = 0; n < MAX_AXIS ; n++ ) {
        range[n] = maximum[n] - minimum[n];
        offset[n] = ( minimum[n] + (range[n]/2) );
        scaling[n] = 1.0 / ( range[n] / 2.0 );
      }
      
    }

    void calcCalibratedMag() {
      getReadings();

      for ( int n = 0; n < MAX_AXIS ; n++ ) {
        calibrated[n] = ( readings[n] - offset[n] )*scaling[n];
      }
    }

    bool magnet_found(float detection_threshold) {
      float magnitude =  sqrt( pow(calibrated[0] , 2) + pow(calibrated[1] , 2) + (calibrated[2] , 2) );
      if ( magnitude > detection_threshold ) {
        return true;
      } 
      else {
        return false;
      }
    }
    
}; // End of Magnetometer_c class definition

#endif
