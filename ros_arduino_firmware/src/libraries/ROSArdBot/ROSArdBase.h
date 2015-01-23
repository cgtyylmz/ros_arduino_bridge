#ifndef ROSARDBASE_H
#define ROSARDBASE_H

// bring in the various definitions of int types such as uint_32, uint_8, etc.
#include <inttypes.h>
#include "ROSArdTypes.h"
#include "BMSerial.h"
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Servo.h>      // can comment out to save 900 bytes if not needed.


#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define PING           'p'
#define RESET_ENCODERS 'r'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define HARDWARE_PID   'h'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define LEFT            1
#define RIGHT           0

class ROSArdBase {

protected:
    bool hasEncoder;
    bool hasMotorController;
    bool hasServos;
    bool hasSensors;
    int  numServos;
    Servo servos[2];            // TODO. This may need to go in seperate class for more flexibility.
    int  numSensors;
    bool softPID;               // use software PID on arduino or hardward PID built into controller board
    long currentLeftSpeed;      // M1 maps to LEFT motor. Actual speed calculated by methods in child classes
    long currentRightSpeed;     // M2 maps to RIGHT motor, add other motors as needed to child classes ??
    int MaxPWM;                 // Arduino supports 8bit PWM, thus maximum duty cycle = 255 normally
    bool moving;                // is the bot currently moving
    int Kp;                     // PID coefficients for software-based PID
    int Kd;
    int Ki;
    int Ko;                     // Setting for 'output' term. See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
    
    int tx;                     // for communications between Arduino and controller board if needed
    int rx;
    long cbBaud;                 // communication rate between Arduino and Controller board if needed

    // A pair of varibles to help parse serial commands (thanks Fergs)
    int arg;
    int index;
    // Variable to hold an input character
    char chr;
    // Variable to hold the current single-character command
    char cmd;
    // Character arrays to hold the first and second arguments
    char argv1[16];
    char argv2[16];
    // The arguments converted to integers
    long arg1;
    long arg2;    
    
    SetPointInfo leftSetPoint;
    SetPointInfo rightSetPoint;

    int servoAddress[16];   // allow up to 16 different servo pins Does this belong in individual subclasses???

public:
    unsigned long nextPID;
    int PID_interval;                // how often to update the software PID in microseconds (30fps = 1000/30 = 33 ms)
    long lastMotorCommand;
    int auto_stop_interval;
    int ROStx_pin;
    int ROSrx_pin;
    long ROSbaud;
    
    ROSArdBase(){
        MaxPWM = 255;    // Arduinos support 8bit PWM, thus max duty cyle = 255
        moving = 0;      // robot is initially not moving
        auto_stop_interval = 2000;
        lastMotorCommand = auto_stop_interval;
        arg = 0;
        index = 0;
        ROSrx_pin = 0;  // default to using hardware Serial port (USB)
        ROStx_pin = 1;  // Alternative hardward ports are 16/17
        ROSbaud = 57600;
    };
    
    ROSArdBase(int set_tx, int set_rx, int cBaud) { 
        tx = set_tx;
        rx = set_rx;
        cbBaud = cBaud;
        MaxPWM = 255;
        moving = 0;
        auto_stop_interval = 2000;
        lastMotorCommand = auto_stop_interval;
        arg = 0;
        index = 0;
        ROSbaud = 57600;
        ROSrx_pin = 0;  // default to using hardware Serial port (USB)
        ROStx_pin = 1;  // Alternative hardward ports are 15/14, 17/16, 19/18
    };
    
    // the following must be implemented by child classes that inherit from this base class
    
    // These require a motorcontroller be attached.
    virtual bool initMotorController() = 0;
    virtual bool setMotorSpeed(int motor_num, int spd) = 0;
    virtual bool setMotorSpeeds(int leftSpeed, int rightSpd) = 0;
    
    // the following require motor encoders
    virtual long readEncoder(int encoder_num)=0;
    virtual bool readEncoders(long &leftEnc, long &rightEnc) = 0;
    virtual long readSpeed(int encoder_num) = 0;        // read speed based on individual encoder
    virtual bool readSpeeds() = 0;                      // read speed of all encoders
    virtual bool resetEncoder(int encoder_num) = 0;     // reset an individual encoder
    virtual bool resetEncoders() = 0;                   // reset all encoders to zero
    virtual void doPID(SetPointInfo * p) = 0;           // PID routine to compute the next motor commands if softPID == TRUE
    virtual void updatePID() = 0;                       // Update encoder values and call PID routine (softPID == TRUE)
    virtual bool updateHardwarePID(int *pid_args) = 0;  // Update PID settings for hardware based PID

    // required if Servos are implemented
    virtual void initServos() = 0;
    virtual void servoPins(int *pinArray) = 0;
    
 
    // generic getters and setters
    /* Command and Control */
    void resetCommand();
    void runCommand(String &result);
    void handleSerial(char c, String &result);
    void sendSerial();
    int serialAvailable();
    
    /* Encoder Methods */
    bool gethasEncoder();           //Does this instance support encoders?
    void sethasEncoder(bool yep);
    long getLeftSpeed();            // current speed of the leftMotor/M1 stored in class variable currentLeftSpeed
    long getRightSpeed();            // these speed values are implemented in child classes based on robot particulars
    
    bool gethasMotorController();   //Does this instance have a motor controller
    void sethasMotorController(bool yep);
    bool isMoving();
    void setMoving(bool yep);
    
    /*  Sensor Methods */
    bool gethasSensors();           //Are there sensors on this instance
    void sethasSensors(bool yep);
    int  getnumSensors();           // Number of Sensors on instance, -1 if bot does not have sensors
    bool setnumSensors(int num);    // Set the number of Sensors on the bot.
    float microsecondsToCm(long microseconds);  // convert microseconds from sonar to centimeters
    long PingSonar(int pin);        // very basic Sonar method. Uses same pin for Trigger and Polling
    
    /* Servo Methods */
    bool gethasServos();            //Are there servos
    void sethasServos(bool yep);
    int  getnumServos();            //Number of Servos
    bool setnumServos(int num);
    
    bool getsoftPID();              // use software PID on the Arduino. Set FALSE
    void setsoftPID(bool yep);      // if no PID or PID is handled by seperate motorcontroller
    int  getPID_interval();              // softPID must be TRUE or returns -1 as error indicator
    bool setPID_interval(int PID_int);  // set in milliseconds, softPID must be TRUE otherwise returns FALSE
    bool resetPID();                // Reset PID state.
   
    int  getMaxPWM();               //
    void setMaxPWM(int max_PWM);
    bool setsoftPIDcoefficients(float P, float I, float D, float O);
    bool getsoftPIDcoefficients(float &P, float &I, float &D, float &O);
    
    /*  Communication Methods */
    int  getTx();
    void setTx(int num);
    int  getRx();
    void setRx(int num);
    long getcbBaud();
    void setcbBaud(int baud);
};

#endif // ROSARDBASE_H

