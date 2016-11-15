#include <Arduino.h>
#include <Wire.h>
#include "I2CFunctions.h"
// We got a Lidar per laser. 

#ifndef LIDAR_OBJECT_H
#define LIDAR_OBJECT_H

enum LIDAR_STATE {
  SHUTING_DOWN = 240,       // Shutdown the laser to reset it
  NEED_RESET = 48,          // Too much outliers, need to reset
  RESET_PENDING = 80,       // Wait 15ms after you reset the Lidar, we are waiting in this state
  NEED_CONFIGURE = 144,     // 15ms passed, we now configure the Lidar
  ACQUISITION_READY = 32,   // I started an acquisition, need someone to read it
  ACQUISITION_PENDING = 64, // The acquisition in on progress
  ACQUISITION_DONE = 128    // I read the data, need to start an acq again
};

class LidarObject {
  public:
/*******************************************************************************
  Constructor
*******************************************************************************/
    LidarObject() {};
/*******************************************************************************
  begin : Begin the I2C master device

  If fasti2c is true, use 400kHz I2C
*******************************************************************************/
    void begin(byte _EnablePin = 2, byte _ModePin = 1, byte _Lidar = 0x62, byte _configuration = 2,char _name = 'A'){
      pinMode(_EnablePin, OUTPUT);
      configuration = _configuration;
      address = _Lidar;
      lidar_state = NEED_RESET;
      EnablePin = _EnablePin;
      ModePin = _ModePin;
      name = _name;
    };


/*******************************************************************************
  setName : set the One char long name

  name is a one char long name 
*******************************************************************************/
    void setName(char _name){
      name = _name;
    };

/*******************************************************************************
  getName : get the One char long name

  return name which is a one char long name 
*******************************************************************************/
    char getName(char _name){
      return name;
    };

/*******************************************************************************
  on : Power On the device
*******************************************************************************/
    void on(){
      digitalWrite(EnablePin, HIGH);
    };

/*******************************************************************************
  off : Power Off the device
*******************************************************************************/
    void off(){
      digitalWrite(EnablePin, LOW);
    };

/*******************************************************************************
  on : Power On the device
*******************************************************************************/
    void enable(){
      digitalWrite(ModePin, HIGH);
    };

/*******************************************************************************
  off : Power Off the device
*******************************************************************************/
    void disable(){
      digitalWrite(ModePin, LOW);
    };

/*******************************************************************************
  timer_update : Update the timer to the current time to start the timer.
*******************************************************************************/
    void timer_update(){
      timeReset = micros();
    }


/*******************************************************************************
  reset_chack : Check the reset timer to see if the laser is correctly resetted

  The laser takes 20ms to reset
*******************************************************************************/
    bool check_reset(){
      if(lidar_state != NEED_RESET)
        return true;

      return (micros() - timeReset > 20000);
    }


/*******************************************************************************
  check_timer : Check the reset timer to see if the laser is correctly resetted

  The laser takes 20ms to reset
*******************************************************************************/
    bool check_timer(){
      if(lidar_state != RESET_PENDING)
        return true;

      return (micros() - timeReset > 20000);
    }

/*******************************************************************************
  resetNacksCount : The nack counter makes the Arduino able to know if a laser 
  needs to be resetted
*******************************************************************************/
    bool resetNacksCount(){
      nacksCount = 0;
    }

    
    byte nacksCount = 0;
    unsigned long timeReset = 0;
    byte configuration = 2;
    byte address = 0x62;
    LIDAR_STATE lidar_state = NEED_RESET;
    byte EnablePin = 2;
    byte ModePin = 1;
    char name;
};

#endif
