#ifndef LIDAR_CONTROLLER_H
#define LIDAR_CONTROLLER_H

#include "I2CFunctions.h"
#include "LidarObject.h"
#include <Wire.h>

// LASER MODES


// Registers are separeted between READ & WRITE registers.
// Indeed the result reading or writing to the same internal register does not affect
// the Lidar the same way
// All registers are not used here, but they are ready to be used in newer versions

// READ Registers
// Those are registers we only READ from
#define STATUS_REGISTER           0x01
#define SIGNAL_STRENGH_REGISTER   0x0e
#define ERROR_REGISTER            0x40
#define MEASURED_VALUE_REGISTER   0x8f
#define READ_SERIAL_REGISTERS     0x96

// WRITE Registers
// Those are register we only WRITE on
#define CONTROL_REGISTER          0x00
#define SERIAL1_REGISTER          0x18
#define SERIAL2_REGISTER          0x19
#define ADDRESS_REGISTER          0x1a
#define PARTY_LINE_REGISTER       0x1e
#define COMMAND_REGISTER          0x40
#define SCALE_VELOCITY_REGISTER   0x45
#define OFFSET_REGISTER           0x13

// Values
#define INITIATE_VALUE            0x04
#define PARTY_LINE_ON             0x00
#define PARTY_LINE_OFF            0x08

// Busyflag from STATUS_REGISTER
#define BUSYFLAG_READY_VALUE      0x00

// Wait between I2C transactions in µs
// One bit every 10µs (2.5µs in 400kHz)
// Wait at least 5 bits to wait for slave answer
#define I2C_WAIT                  50
#define FORCE_RESET_OFFSET        true

#define PRINT_DEBUG_INFO          false
#define LIDAR_TIMEOUT_MS          20
#define MAX_LIDARS                8
#define MAX_NACKS                 10

class LidarController {
  public:
    /*******************************************************************************
      begin :
      Start the I2C line with the correct frequency
    *******************************************************************************/
    void begin(bool fasti2c = false) {
 
      for(byte i=0; i<MAX_LIDARS; i++){
        distances[i] = 0;
        nacks[i] = 0;
        statuses[i]=0;
      }

      Wire.begin();
      if (fasti2c) {
#if ARDUINO >= 157
        Wire.setClock(400000UL); // Set I2C frequency to 400kHz, for the Due
#else
        TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
      }
    }

    /*******************************************************************************
      add :
      Add a new Lidar and use resetLidar : It assure the lidar is NOT on the 0x62 line
    *******************************************************************************/
    bool add(LidarObject* _Lidar, byte _id) {
      if (_id >= MAX_LIDARS)
        return false;
      lidars[_id] = _Lidar;
      resetLidar(_id);
      count++;
      return true;
    }

    /*******************************************************************************
      configure : Configure the default acquisition mode

      configuration : The configuration of the Lidar
          - 0 = basic configuration
          - 1 = faster (Do not read 3 times), bit noisier
          - 2 = low noise, low sensitivity, less false detection (Default)
          - 3 = High noise, high sensitivity
      Lidar : Address of the Lidar (0x62 by default)
    *******************************************************************************/
    void configure(byte Lidar = 0, byte configuration = 2) {
      byte nack = 0;
      switch (configuration) {
        case 0: //  Default configuration
          nack = I2C.write(lidars[Lidar]->address, 0x00, 0x00);
          break;
        case 1: //  Set aquisition count to 1/3 default value, faster reads, slightly
          //  noisier values
          nack = I2C.write(lidars[Lidar]->address, 0x04, 0x00);
          break;
        case 2: //  Low noise, low sensitivity: Pulls decision criteria higher
          //  above the noise, allows fewer false detections, reduces
          //  sensitivity
          nack = I2C.write(lidars[Lidar]->address, 0x1c, 0x20);
          break;
        case 3: //  High noise, high sensitivity: Pulls decision criteria into the
          //  noise, allows more false detections, increses sensitivity
          nack = I2C.write(lidars[Lidar]->address, 0x1c, 0x60);
          break;
      }
      shouldIncrementNack(Lidar, nack);
    };

    /*******************************************************************************
      changeAddress : Change the address of one Lidar

      returns 0 if success
            1 if error writing the serial number (byte 1)
            2 if error writing the serial number (byte 2)
            3 if error sending the Lidar address
            4 if error disabling the main address
            5 if the new Lidar address is already ON
            6 if the Lidar do not respond
    *******************************************************************************/
    byte changeAddress(byte Lidar) {
      byte _lidar_new = lidars[Lidar]->address;
      byte nack = 0;
      // Return 6 = The device do not respond
      if (!I2C.isOnline(0x62)){
        shouldIncrementNack(Lidar, 1); // If we set anything else than 0, it increments
        return 6;
      }
      // Return 5 = We already got an I2C device at this place
      if (I2C.isOnline(_lidar_new)){
        shouldIncrementNack(Lidar, 1);
        return 5;
      }
      /* Serial number part */
      unsigned char serialNumber[2];
      shouldIncrementNack(Lidar, I2C.readWord(0x62, 0x96, serialNumber));
      // Return 1 = Error sending the Serial (byte 1)
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x18, serialNumber[0])))
        return 1;
      // Return 2 = Error sending the Serial (byte 2)
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x19, serialNumber[1])))
        return 2;

      // Return 3 = Error sending the Lidar address
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x1a, _lidar_new)))
        return 3;

      // Return 4 = Error disabling the Lidar Main address (0x62)
      if (shouldIncrementNack(Lidar, I2C.write(0x62, 0x1e, 0x08)))
        return 4;

      return 0;
    };

    /*******************************************************************************
      status : check the status of the Lidar

      returns the status register (0x01 for version 21 of the Lidar Software)
    *******************************************************************************/
    byte status(byte Lidar = 0) {
      byte data[1] = {171}; // Initializing with a non 0 NOR 1 data to ensure we got
      // no interference
      byte nack = I2C.readByte(lidars[Lidar]->address, 0x01, data);
      shouldIncrementNack(Lidar, nack);
      return data[0];
    };

    /*******************************************************************************
      async : start an acquisition
                  - with preamp enabled
                  - with DC stabilization

      returns the nack error (0 if no error)
    *******************************************************************************/
    byte async(byte Lidar = 0) {
      byte nack = I2C.write(lidars[Lidar]->address, 0x00, 0x04);
      shouldIncrementNack(Lidar, nack);
    };

    /*******************************************************************************
      distance :
        - Read the measured value from data registers
    *******************************************************************************/
    int distance(byte Lidar, int * data) {
      byte distanceArray[2];
      byte nackCatcher = I2C.readWord(lidars[Lidar]->address, 0x8f, distanceArray);
      shouldIncrementNack(Lidar, nackCatcher);
      int distance = (distanceArray[0] << 8) + distanceArray[1];
      *data = distance;
      return nackCatcher;
    };

    /*******************************************************************************
      setState : Change the status of the Lidar Object
    *******************************************************************************/
    void setState(byte Lidar = 0, LIDAR_STATE _lidar_state = NEED_RESET) {
      lidars[Lidar]->lidar_state = _lidar_state;
    };

    /*******************************************************************************
      getState : Get the status of the Lidar Object
    *******************************************************************************/
    LIDAR_STATE getState(byte Lidar = 0) {
      return lidars[Lidar]->lidar_state;
    };

    /*******************************************************************************
      setOffset : Set an offset to the Lidar
    *******************************************************************************/
    void setOffset(byte Lidar, byte data) {
        I2C.write(lidars[Lidar]->address, OFFSET_REGISTER, data);
    };

    /*******************************************************************************
      distanceAndAsync : Get the distance then start a new acquisition

      We could use the async() method in ACQUISITION_DONE, but it would need to spin
      one time more before starting the acquisition again
    *******************************************************************************/
    byte distanceAndAsync(byte Lidar, int * data) {
      byte nackCatcher = distance(Lidar, data);
      // if error reading the value, try ONCE again
      if (nackCatcher)
        distance(Lidar, data);
      // Start a new acquisition
      async(Lidar);
      return nackCatcher;
    };

    /*******************************************************************************
      resetLidar :
        * set the Power Enable pin to 0
        * set the Need Reset state to be reinitialized 20ms later
    *******************************************************************************/
    void resetLidar(byte Lidar = 0) {
      lidars[Lidar]->off();
      lidars[Lidar]->timer_update();
      setState(Lidar, SHUTING_DOWN);
    };

    /*******************************************************************************
      preReset :
        * set the reset latch (resetOngoing) to true to prevent starting 2 lidars simultaneously
        * set the lidar on & start the 16 µS timer
    *******************************************************************************/
    void preReset(byte Lidar = 0) {
      resetOngoing = true;
      lidars[Lidar]->on();
      lidars[Lidar]->timer_update();
    };


    /*******************************************************************************
      getCount :
        * returns the count of the lasers
    *******************************************************************************/
    byte getCount(){
      return count;
    };

    /*******************************************************************************
      postReset :
        * change the lidar address
        * stop the reset ongoing
    *******************************************************************************/
    void postReset(byte Lidar = 0) {
      changeAddress(Lidar);
      resetOngoing = false;
    };


    /*******************************************************************************
      shouldIncrementNack : increments the nacksCount if nack happens
    *******************************************************************************/
    byte shouldIncrementNack(byte Lidar = 0, byte nack = 0){
      if(nack){
        lidars[Lidar]->nacksCount += 1;
        nacks[Lidar] = nack;
      }
      return nack;
    };

    /*******************************************************************************
      checkNacks : Returns if the laser needs or not a reset
        if have to be resetted, reset the counter and return true. The setState
        instruction have to be in the spinOnce function
    *******************************************************************************/
    bool checkNacks(byte Lidar = 0){
      if(lidars[Lidar]->nacksCount > MAX_NACKS){
        lidars[Lidar]->resetNacksCount();
        return true;
      }
      return false;
    };

    /*******************************************************************************
      spinOnce : Main routine to simplify everything in ASYNC mode, checking the
      status of the Lidar for each loop.
        Cases :
          * NEED_CONFIGURE => Configure the Lidar acquisition mode
            -> Go to ACQUISITION_READY
          * ACQUITION_READY => Start the acquisition using async();
            -> Go to ACQUISITION PENDING
          * ACQUISITION_PENDING => Checking the busyFlag
            Get the data and store it in distances
            -> Go to ACQUISITION_READY
          * ACQUISITION_DONE => NOT_USED
          * NEED_RESET => The Lidar is OFF and waits to be started
          * RESET_PENDING => The Lidar is ON, after being OFF and waits 16 µS to be
          ready. No other laser can be on at this time
    *******************************************************************************/
    void spinOnce() {
      // Handling routine
      //for (int8_t i = count - 1; i >= 0; i--) {
      for(uint8_t i = 0; i<count; i++){
#if PRINT_DEBUG_INFO
        Serial.print("Laser ");
        Serial.print(i);
#endif
        switch (getState(i)) {
          case NEED_CONFIGURE:
#if PRINT_DEBUG_INFO
            Serial.println(" NEED_CONFIGURE");
#endif
            configure(i);
            setState(i, ACQUISITION_READY);
            break;
          case ACQUISITION_READY:
#if PRINT_DEBUG_INFO
            Serial.println(" ACQUISITION_READY");
#endif
            async(i);
            setState(i, ACQUISITION_PENDING);
            break;
          case ACQUISITION_PENDING:
#if PRINT_DEBUG_INFO
            Serial.println(" ACQUISITION_PENDING");
#endif
            // Get the status bit, if 0 => Acquisition is done
            if (bitRead( status(i), 0) == 0) {
              int data = 0;
              distance(i, &data);
#if PRINT_DEBUG_INFO
              Serial.println(i);
              Serial.println(data);
#endif
              nacks[i] = nacks[i] & 0b00000111; // Remove the bit
              if((abs(data - distances[i]) > 100) | (data < 5 or data > 1000)){
                shouldIncrementNack(i, 1);
                nacks[i] = 8 | nacks[i]; // Set the suspicious data bit
              }
              // Write data anyway but the information is send via nacks = 15
              distances[i] = data;
              setState(i, ACQUISITION_DONE);
            }
            break;
          case ACQUISITION_DONE:

#if PRINT_DEBUG_INFO
            Serial.println(" ACQUISITION_DONE");
#endif
#if FORCE_RESET_OFFSET
              setOffset(i, 0x00);
              setState(i, ACQUISITION_READY);
#endif
            break;
          case NEED_RESET:
#if PRINT_DEBUG_INFO
            Serial.println(" NEED_RESET");
#endif
            if (!resetOngoing) {
              preReset(i);
              setState(i, RESET_PENDING);
            }
            break;
          case RESET_PENDING:
#if PRINT_DEBUG_INFO
            Serial.println(" RESET_PENDING");
#endif
            // Check the timer, if done, laser is ready to reset, change state
            if (lidars[i]->check_timer()) {
              postReset(i);
              setState(i, NEED_CONFIGURE);
            }
            break;

          case SHUTING_DOWN :
            if (lidars[i]->check_timer()) {
              postReset(i);
              setState(i, NEED_RESET);
            }
            break;
          default:
            break;
        } // End switch case

        if(checkNacks(i)){
           resetLidar(i);
        }
        statuses[i] = (getState(i) & 0xF0) | (nacks[i] & 0x0F);
      } // End for each laser
    };



    int distances[MAX_LIDARS];
    int nacks[MAX_LIDARS];
    uint8_t statuses[MAX_LIDARS];
  private:
    bool resetOngoing = false;
    LidarObject* lidars[MAX_LIDARS];
    byte count = 0;
};


#endif
