#ifndef ROSARDROBOCLAW_H
#define ROSARDROBOCLAW_H
// bring in the virtual bass class from which all controllers are derived.
#include <inttypes.h>
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "ROSArdBase.h"
#include "ROSArdTypes.h"
#include "BMSerial.h"
#include "RoboClaw.h"

class ROSArdRoboclaw: public ROSArdBase {
  public:
    int rcAddress;     // address of the controller.
    RoboClaw *RCmc;    // instance of the RoboClaw controller object.
    VPID leftMotor;    // PID settings for onboard PID controller of RoboClaw
    VPID rightMotor;   // P,I,D, qpps, qpps = quadrature pulses per second

    ROSArdRoboclaw(RoboClaw *mc, int address, long baud)  {
        // Variables specific to this class
        RCmc = mc;              // instance of RoboClaw class
        rcAddress = address;    // address of RoboClaw (allow use of multiple controllers)
        // Variables in RosArdBase class
        cbBaud = baud;          // set controller board baud rate. (Between Arduino and RoboClaw)
        PID_interval = 33;      // 30hz (33ms per hz), used for soft PID only.
        nextPID = PID_interval; // initially set the next PID update to the interval
    }
    // These methods are specific to the RoboClaw motorcontroller.
    bool setRcPIDvalues(VPID *p, int motor); // settings for Roboclaw hardware PID controller
    bool getRcPIDvalues(VPID *p, int motor); // stored in class structures leftMotor and rightMotor
    bool sendPIDtoRC(int motor);             // send Values of leftMotor or rightMotor to Roboclaw 
    bool getPIDfromRC(VPID *p, int motor);   // retrieve settings actuallyon RoboClaw
    
    // The following must be implemented to conform to the base ROSArdbase class
    long readEncoder(int encoder_num);
    bool readEncoders(long &leftEnc, long &rightEnc);
    bool setMotorSpeed(int motor_num, int spd);
    bool setMotorSpeeds(int leftSpd, int rightSpd);
    bool resetEncoders();                  // reset all encoders to zero
    bool resetEncoder(int encoder_num);    // reset an individual encoder (not implemented on RoboClaw, returns false
    long readSpeed(int encoder_num);       // read speed based on individual encoder
    bool readSpeeds();                     // read speed of all encoders
    bool initMotorController();
    void initServos() {};                  // no servos on this bot
    void servoPins(int *pinArray){};       // no servos on this bot
    void doPID(SetPointInfo *p);           // On RoboClaw we're allowing the mc board to handle velocity PID at 300hz
    void updatePID();
    bool updateHardwarePID(int *pid_args);
};
#endif
