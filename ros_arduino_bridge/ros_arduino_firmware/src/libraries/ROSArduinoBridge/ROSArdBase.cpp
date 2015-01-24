#include "ROSArdBase.h"
#include <Servo.h>      // can comment out to save 900 bytes if not needed.

bool ROSArdBase::gethasEncoder() {
    return hasEncoder;
}

void ROSArdBase::sethasEncoder(bool yep) {
    hasEncoder = yep;
}

bool ROSArdBase::gethasMotorController() {
    return hasMotorController;
}

void ROSArdBase::sethasMotorController(bool yep) {
    hasMotorController = yep;
}

/** isMoving
* Is the robot moving?
* @return bool
*/
bool ROSArdBase::isMoving() {
  return moving;
}

/** setMoving
* Setter for wether the robot is moving or not.
* @param  bool
*/
void ROSArdBase::setMoving(bool yep) {
  moving = yep;
}

bool ROSArdBase::gethasSensors() {
    return hasSensors;
}

void ROSArdBase::sethasSensors(bool yep) {
    hasSensors = yep;
}

/** getnumSensors
 *
 *  Return number of Sensors attached to robot if robot has sensors
 * returns -1 if robot isn't set up for sensors
 * @return  int     Number of sensors or -1 for not set up.
 */
int ROSArdBase::getnumSensors() {
    if (hasSensors) {
        return numSensors;
    } else {
        // instance doesn't have sensors return -1 indicating 'false' condition
        return -1;
    }
}

/** setnumSensors
 * 
 * Set the number of sensors assuming that the robot has its
 * hasSensors flag as TRUE
 * @return  bool    True if set, False = robot not setup for sensors
 */
bool ROSArdBase::setnumSensors(int num) {
    if (hasSensors) {
        numSensors = num;
        return true;
    } else {
        // instance doesn't have sensors, return false
        return false;
    }
}

bool ROSArdBase::gethasServos() {
    return hasServos;
}

void ROSArdBase::sethasServos(bool yep) {
    hasServos = yep;
}

/** getnumServos
 *
 * Return number of Servos attached to robot if robot has servos
 * returns -1 if robot isn't set up for servos
 * @return  int     Number of Servos or -1 for not set up.
 */
int ROSArdBase::getnumServos() {
    if (hasServos) {
        return numServos;
    } else {
        // instance doesn't have sensors return -1 indicating 'false' condition
        return -1;
    }
}

/** setnumSensors
 *
 * Set the number of servos assuming that the robot has it's
 * hasServos flag as TRUE
 * @return  bool    True if set, False = robot not setup for servos
 */
bool ROSArdBase::setnumServos(int num) {
    if (hasServos) {
        numServos = num;
        return true;
    } else {
        // instance doesn't have sensors, return false
        return false;
    }
    
}

/** getsoftPID
 * Is software PID used on this robot?
 * @return  bool
 */
bool ROSArdBase::getsoftPID() {
    return softPID;
}

/** setsoftPID
 * Set to TRUE if Arduino is to perform PID in software, false if not needed
 * or if hardware PID is used instead. (if PID is built into motor controller)
 */
void ROSArdBase::setsoftPID(bool yep) {
     softPID = yep;
}

/** getLeftSpeed
 *
 * Get the most recent speed of the left motor. Setting is performed in child 
 * classes because how speed is determined will vary from robot to robot
 */
long ROSArdBase::getLeftSpeed() {
    return currentLeftSpeed;
}

/** getLeftSpeed
 *
 * Get the most recent speed of the left motor. Setting is performed in child
 * classes because how speed is determined will vary from robot to robot
 */
long ROSArdBase::getRightSpeed() {
    return currentRightSpeed;
}

/** getMaxPWM
 * Get maximum PWM rate of Arduino. As of 2014, most/all Arduinos use 8bit PWM
 * and thus have a maximum duty cycle of 255.
 * @return  int     Maximum duty cycle of Arduino PWMs
 */
int ROSArdBase::getMaxPWM() {
    return MaxPWM;
}

/** setMaxPWM
 * Set maximum duty cycle. On arduinos, it's assumed to be 8bit and thus 255
 */
void ROSArdBase::setMaxPWM(int max_PWM) {
    MaxPWM = max_PWM;
}

/** getPID_interval
 * What rate in milliseconds does the softare PID implmented on the Arduino
 * update? If softPID must be set to TRUE otherwise returns -1 as error indicator
 * @return  int
 */
int ROSArdBase::getPID_interval() {
    if (softPID == true) {
        return PID_interval;
    } else {
        return -1;
    }
}

/** setPID_interval
 * Set PID rate in milliseconds. softPID must be true or will return false
 * @param   int     PID_rate    How frequently the PID routine is updated in milliseconds.
 * @return  bool                True on success, false if not using softPID
 */
bool ROSArdBase::setPID_interval(int PID_int) {
    if (softPID == true) {
        PID_interval = PID_int;
    } else {
        return false;
    }
}

int ROSArdBase::getTx() {
    return tx;
}

int ROSArdBase::getRx() {
    return rx;
}

long ROSArdBase::getcbBaud() {
  return cbBaud;
}

void ROSArdBase::setTx(int pin) {
  tx = pin;
}

void ROSArdBase::setRx(int pin) {
  rx = pin;
}

void ROSArdBase::setcbBaud(int baud) {
  cbBaud = baud;
}

/** microsecondsToCm
 * Generic method to convert time to distance using Sonars
 * assumes sealevel and average room temperatures.
 *
 * @param   long    microseconds    How long was the pulse?
 * @param   float                   Distance in centimeters
 */
float ROSArdBase::microsecondsToCm(long microseconds) {
    // The speed of sound is 340 m/s or 29 microseconds per cm.
    // The ping travels out and back, so to find the distance of the
    // object we take half of the distance travelled.
    //
    // 1/(29*2) = 0.017241379310345. Multiplication is 52x faster than division on
    // Arduino. See http://forum.arduino.cc/index.php?topic=92684.0
    return microseconds * 0.017241379310345;
}

/** PingSonar
 *
 * Blocking Sonar Ping. Inclued for completeness from original library
 * but you will proabaly want to overload this or write your own sensor
 * methods in child classes. This method assumes that the sonar trigger 
 * and response exsits on the same pin. This is often not the case.
 *
 * @param   int     pin     Which arduino pin does the Sonar live on.
 * @return  long            range in centimeters.
 */
long ROSArdBase::PingSonar(int pin) {
    long duration, range;
    
    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
    
    // The same pin is used to read the signal from the PING))): a HIGH
    // pulse whose duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    pinMode(pin, INPUT);
    duration = pulseIn(pin, HIGH);
    
    // convert the time into meters
    range = microsecondsToCm(duration);
    
    return(range);
}

/** resetPID
 *
 * Used primarily for software PID on the arduino. Not needed when using hardware PID on roboclaw
 * controller. hasEncoder must be TRUE to perform actions.
 *
 * @return bool
 */
bool ROSArdBase::resetPID() {
  if(hasEncoder) {
    /*
     * Initialize PID variables to zero to prevent startup spikes
     * when turning PID on to start moving
     * In particular, assign both Encoder and PrevEnc the current encoder value
     * See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
     * Note that the assumption here is that PID is only turned on
     * when going from stop to moving, that's why we can init everything on zero.
     */
    leftSetPoint.TargetTicksPerFrame = 0.0;
    leftSetPoint.Encoder = readEncoder(LEFT);
    leftSetPoint.PrevEnc = leftSetPoint.Encoder;
    leftSetPoint.output = 0;
    leftSetPoint.PrevInput = 0;
    leftSetPoint.ITerm = 0;
    
    rightSetPoint.TargetTicksPerFrame = 0.0;
    rightSetPoint.Encoder = readEncoder(RIGHT);
    rightSetPoint.PrevEnc = rightSetPoint.Encoder;
    rightSetPoint.output = 0;
    rightSetPoint.PrevInput = 0;
    rightSetPoint.ITerm = 0;
  } else {
    return false;
  }
}

/** setsoftPIDcoefficients
*
* Set the coefficeients for the software implementation of velocity PID
* @param    int     P
* @param    int     I
* @param    int     D
* @param    int     O   'Output' coefficeient
* @return   bool        True on success, False on fail
*/
bool ROSArdBase::setsoftPIDcoefficients(float P, float I, float D, float O) {
    if (softPID == true) {
        Kp = P;
        Ki = I;
        Kd = D;
        Ko = O;
        return true;
    } else {
        return false;
    }
}

/** getsoftPIDcoefficients
 *
 * Set the coefficeients for the software implementation of velocity PID
 * @param    int&     P
 * @param    int&     I
 * @param    int&     D
 * @param    int&     O   'Output' coefficeient
 * @return   bool        True on success, False on fail
*/
bool ROSArdBase::getsoftPIDcoefficients(float &P, float &I, float &D, float &O) {
    if (softPID == true) {
        P = Kp;
        I = Ki;
        D = Kd;
        O = Ko;
        return true;
    } else {
        // if softPID is not selected, then return false
        return false;
    }
}

void ROSArdBase::resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/** runCommand
*
* Once a serial message has been received from ROS, execute it
* This converts arguments received by ROS as ASCII strings into long ints
* @return void
*/
void ROSArdBase::runCommand(String &result) {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1); // convert number sent as string to integer
  arg2 = atoi(argv2);

  switch(cmd) {
    case GET_BAUDRATE:
      result += ROSbaud;
      break;
    case ANALOG_READ:
      result += analogRead(arg1);
      break;
    case DIGITAL_READ:
      result += digitalRead(arg1);
      break;
    case ANALOG_WRITE:
      analogWrite(arg1, arg2);
      result = "OK";
      break;
    case DIGITAL_WRITE:
      if (arg2 == 0) digitalWrite(arg1, LOW);
      else if (arg2 == 1) digitalWrite(arg1, HIGH);
      result = "OK";
      break;
    case PIN_MODE:
      if (arg2 == 0) pinMode(arg1, INPUT);
      else if (arg2 == 1) pinMode(arg1, OUTPUT);
      result = "OK";
      break;
    case PING:
      if(hasSensors) {
        result += PingSonar(arg1);
      } else {
        result = "Disabled";
      }
      break;
    case SERVO_WRITE:
      if (hasServos) {
          servos[arg1].write(arg2);
          result = "OK";
      } else {
        result = "Disabled";
      }
      break;
    case SERVO_READ:
      if (hasServos) {
        result += servos[arg1].read();
      } else {
        result = "Disabled";
      }
      break;
    case READ_ENCODERS:
      if(hasEncoder) {
        result += readEncoder(LEFT);
        result += " ";
        result += readEncoder(RIGHT);
      } else {
        result = "Disabled";
      }
      break;
    case RESET_ENCODERS:
      if(hasEncoder) {
        resetEncoders();
        resetPID();
        result = "OK";
      } else {
        result = "Disabled";
      }
      break;
    case MOTOR_SPEEDS:
      if(hasMotorController) {
        /* Reset the auto stop timer */
        lastMotorCommand = millis();
        if (arg1 == 0 && arg2 == 0) {
          setMotorSpeeds(0, 0);
          moving = 0;
        }
        else moving = 1;
        leftSetPoint.TargetTicksPerFrame = arg1;
        rightSetPoint.TargetTicksPerFrame = arg2;
        result = "OK";
      } else {
        result = "Disabled";
      }
      break;
    case HARDWARE_PID:
      // the following is a virtual method required by the base class
      // it will be implemented in a particular motor controller class 
      // as each motor controller will implmenet PID differently if at all.
      updateHardwarePID(pid_args);
      break;
    case UPDATE_PID:
      if (softPID) {
        while ((str = strtok_r(p, ":", &p)) != '\0') {
           pid_args[i] = atoi(str);
           i++;
        }
        Kp = pid_args[0];
        Kd = pid_args[1];
        Ki = pid_args[2];
        Ko = pid_args[3];
        result = "OK";
      } else {
        result = "Disabled";
      }
      break;
    default:
      result = "Invalid Command";
      break;
    }
}

/** handleSerial
*
* If a command is received over the serial interface, parse it
* and execute the relevant commands. For ease of testing in the
* Arduino environment, "!" can be sent at the end of a command from
* the serial monitor. Otherwise a Carriage Return is expected to 
* terminate a command.
* @return void
*/
void ROSArdBase::handleSerial(char c, String &result) {

    chr = c;  // sort the character read in the class variable 'chr'
    
    // Terminate a command with a CR or "!", allows easy testing from Arduino IDE
    if (chr == 13 || chr == 33) {
        if (arg == 1) argv1[index] = NULL;      // set the end of the char array to NULL (terminates strings in C)
        else if (arg == 2) argv2[index] = NULL;
        // not that we've gotten a command terminator, execute the command. Pass 'results' by reference
        runCommand(result);
        // command is done, reset all the variables to await the next command
        resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
        // Step through the arguments
        if (arg == 0) {
          arg = 1;
        } else if (arg == 1)  {
            argv1[index] = NULL;
            arg = 2;
            index = 0;
        } else if (arg ==2) {
          // shouldn't be here. perhaps return an 'invalid' result ??
          // somebody tried to pass three arguments. as of 2014-09-28 longest command
          // only uses 2 arguments.
        }
    }
    else {
        if (arg == 0) {
          // The first arg is the single-letter command
          cmd = chr;
        }
        else if (arg == 1) {
          // Subsequent arguments can be more than one character.
          // These ASCII characters are stored in arguments arrays to later be 
          // converted to INTs as need be.
//TODO code should be augmeted to handle floats as PID coefficients can be floats depending on implementation
          argv1[index] = chr;
          index++;
        }
        else if (arg == 2) {
          argv2[index] = chr;
          index++;
        }
    }
}
