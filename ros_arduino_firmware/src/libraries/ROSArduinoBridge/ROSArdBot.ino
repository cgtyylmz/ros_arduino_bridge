// bring in the virtual bass class from which all controllers are derived.
#include "ROSArdRoboclaw.h"
#include "ROSArdTypes.h"
// SoftSerial instead??
#include "BMSerial.h"
#include "RoboClaw.h"
#include <Servo.h>      // can comment out to save 900 bytes if not needed.

// set up roboclaw object on RX 10, TX 11 with 10ms of timeout (between Arduino and RC)
RoboClaw mc(10,11,10000,true);
BMSerial comPort(0,1); // 16/17 for bluetooth, 0,1 for USB

// Set up the hardward PID structure for roboclaw motor controller
VPID M1_pid = {1.0, .5, 0, 7457};  // M1 (right motor) runs a bit slower than M2
VPID M2_pid = {1.0, .5, 0, 7563};  // at full speed.

// pass in a RoboClaw motorcontroller instance along with address and baud rate between
// Arduino and Roboclaw controller. (This baud is seperate from that between the Arduino 
// and the ROS computer.
ROSArdRoboclaw bot(&mc,0x80,38400);

void setup() {
  bool valid;
  pinMode(14,OUTPUT);
  pinMode(15,OUTPUT);
  digitalWrite(14,HIGH);
  digitalWrite(15,LOW);
  bot.ROSbaud = 57600;
  bot.auto_stop_interval = 4000;
  
  // start serial connection to ROS computer
  comPort.begin(bot.ROSbaud);
  comPort.println("");comPort.println("");
  comPort.print(F("AM: Serial connection to computer started at "));
  comPort.println(bot.ROSbaud);
  
  // set the values for the hardware PID. Must be done before the initMotorController
  // command is given as initMC will set the values to the controller as part of setup.
  valid =  bot.setRcPIDvalues(&M1_pid, LEFT);
  valid &= bot.setRcPIDvalues(&M2_pid, RIGHT);
  if(valid) {
    comPort.println(F("AM: Successfully sent PID info to class variables"));
  }    
  // initialize the controller. Sets default values and fires up the 
  // connection between the arduino and the roboclaw controller
  if(bot.initMotorController()) {
    comPort.println(F("AM: Successfully started up RoboClaw motor controller"));
  } else {
    comPort.println(F("AM: FAILED to startup RoboClaw motor controller !!"));
  }
  bot.isMoving() ? comPort.println(F("AM: Robot is moving!!")) : comPort.println(F("AM: Robot is not moving"));
  delay(3000);
}

void loop() {
  // see if a command has arrived over the serial port. If so
  // execute the command. This contains a blocking while loop
  // which blocks until entire serial buffer is read.
    String results;
    while (comPort.available() > 0) {
        char c = comPort.read();
        bot.handleSerial(c, results);
    }
    if(results.length() > 0) {
      comPort.println(results);
    }
  
    if(bot.gethasEncoder()) {
      if (millis() > bot.nextPID) {
        bot.updatePID();
        bot.nextPID += bot.PID_interval;
      }
      if ((millis() - bot.lastMotorCommand) > bot.auto_stop_interval) {
        bot.setMotorSpeeds(0,0);
        bot.setMoving(false);
      }
    }
}

