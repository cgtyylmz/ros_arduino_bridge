#include "ROSArdRoboclaw.h"

//#define DEBUG

/** readEncoder
*
* Roboclaw reads each quadrature encoder individually
*
* @param   int  encoder_num  Which encoder to retrieve the values for
* @return  long              Signed value of the current encocoder count
*/
long ROSArdRoboclaw::readEncoder(int encoder_num) {
    if (hasEncoder == true) {
        if (encoder_num == LEFT) {
            return RCmc->ReadEncM1(rcAddress);
        } else if (encoder_num == RIGHT) {
            return RCmc->ReadEncM2(rcAddress);
        } else {
            // Roboclaw only has two encoder channels
            return -1;
        }
    } else {
        // hasEncoder is set to FALSE, return -1 as error indicator
        return -1;
    }
}

/** readEncoder
*
* Return values of both encoders by reference.
*
* @param   &long  leftEnc    Address to store value of leftEncoder in
* @param   &long  rightEnc   Address to store value of rightEnc in
* @return  void
*/
bool ROSArdRoboclaw::readEncoders(long &leftEnc, long &rightEnc) {
    if (hasEncoder == true) {
        leftEnc =  RCmc->ReadEncM1(rcAddress);
        rightEnc = RCmc->ReadEncM2(rcAddress);
        return true;
    } else {
        // hasEncoder is set to FALSE
        return false;
    }
}

/** resetEncoders
 * Reset all motor encoders
 * On roboclaw, this is the only option. There is no command for resetting
 * each individual encoder
 *
 * @return  bool            True if successfully reset encoders
 */
bool ROSArdRoboclaw::resetEncoders() {
    if (hasEncoder == true) {
        return RCmc->ResetEncoders(rcAddress);
    } else {
        return false;
    }
}

/** resetEncoder
 *
 * Reset individul encoder. This isn't implmented on the RoboClaw controller.
 * @return  bool            Returns FALSE since RoboClaw doesn't implement this functionality
 */
bool ROSArdRoboclaw::resetEncoder(int encoder_num){
    // roboclaw doesn't allow for reseting individual encoders. It's all or nothing.
    return false;
}

/** setMotorSpeed
*
* Set individual motor speed for either the left or the right channel
*
* @param  int  leftSpd    Left Moter Speed. (M1 on Roboclaw)
* @param  int  rightSpd   Right Motor Speed (M2 on Roboclaw)
* @return  void
*/
bool ROSArdRoboclaw::setMotorSpeed(int motor_num, int spd) {
    if (hasMotorController == true) {
        if (motor_num == LEFT) {
            return RCmc->SpeedM1(rcAddress,spd);
        } else if (motor_num == RIGHT) {
            return RCmc->SpeedM2(rcAddress,spd);
        }
    } else {
        return false;
    }
}

/**  setMotorSpeeds
*
* Set the LEFT and RIGHT motor speed based on qpps values
*
* @param   int  leftSpd    Left Moter Speed. (M1 on Roboclaw)
* @param   int  rightSpd   Right Motor Speed (M2 on Roboclaw)
* @return  bool            Returns false if hasMotorController == FALSE
**/
bool ROSArdRoboclaw::setMotorSpeeds(int leftSpd, int rightSpd) {
  // roboclaw allows you to send speed commands to both motors at the same time.
    if (hasMotorController == true) {
        return RCmc->SpeedM1M2(rcAddress, leftSpd, rightSpd);
    } else {
        return false;
    }
}

/** readSpeed
 * @param    int     motor_num     Read either M1(LEFT) or M2(RIGHT) from Roboclaw
 * @return   long                    speed of motor in qpps
 */
long ROSArdRoboclaw::readSpeed(int motor_num) {
    if (hasEncoder) {
        if (motor_num == LEFT) {
          return RCmc->ReadSpeedM1(rcAddress);
        } else if (motor_num == RIGHT) {
          return RCmc->ReadSpeedM2(rcAddress);
        } else {
          return -1;
        }
    } else {
        return -1;
    }
}

/** readSpeeds
 *
 * Read the speed of each motor controller. The Roboclaw can provide
 * speed information on 2 motors. Speed is stored in Class variables M1speed
 * and M2speed
 *
 * @return  bool
 */
bool ROSArdRoboclaw::readSpeeds() {
    if (hasEncoder) {
        currentLeftSpeed    = readSpeed(LEFT);
        currentRightSpeed   = readSpeed(RIGHT);
        return true;
    } else {
        return false;
    }
}

/** setRcPID
 * Set the hardware PID settings for roboclaw
 * PID settings can be different for each motor that is being controlled
 *
 * @param object    VPID *p     VPID structure containing settings for a motor
 * @return bool
 */
bool ROSArdRoboclaw::setRcPIDvalues(VPID *p, int motor) {
    if (motor == LEFT) {
        leftMotor.P = p->P;
        leftMotor.I = p->I;
        leftMotor.D = p->D;
        leftMotor.qpps = p->qpps;
    } else if (motor == RIGHT) {
        rightMotor.P = p->P;
        rightMotor.I = p->I;
        rightMotor.D = p->D;
        rightMotor.qpps = p->qpps;
    } else {
        return false;
    }
    return true;
}

/** getRcPIDvalues
*
* Get the hardward PID settings stored in the class variable leftMotor or
* rightMotor. These are not necessarily the values active on the motor
* controller.
*/
bool ROSArdRoboclaw::getRcPIDvalues(VPID *p, int motor) {
    if (motor == LEFT) {
        p->P = leftMotor.P;
        p->I = leftMotor.I;
        p->D = leftMotor.D;
        p->qpps = leftMotor.qpps;
    } else if (motor == RIGHT) {
        p->P = rightMotor.P;
        p->I = rightMotor.I;
        p->D = rightMotor.D;
        p->qpps = rightMotor.qpps;
    } else {
        return false;
    }
    return true;
}

bool ROSArdRoboclaw::sendPIDtoRC(int motor) {
    bool valid;
    if (motor == LEFT) {
      valid = RCmc->SetM1VelocityPID(rcAddress, leftMotor.D, leftMotor.P, leftMotor.I, leftMotor.qpps);
#ifdef DEBUG
Serial.print("sendPIDtoRC LEFT:");Serial.println(valid,DEC);
#endif
    } else if (motor == RIGHT) {
      valid = RCmc->SetM2VelocityPID(rcAddress, rightMotor.D, rightMotor.P, rightMotor.I, rightMotor.qpps);
#ifdef DEBUG
Serial.print("sendPIDtoRC RIGHT:");Serial.println(valid,DEC);
#endif
    } else {
        return false;
    }
    return valid;
}

bool ROSArdRoboclaw::getPIDfromRC(VPID *p, int motor) {
    bool valid;
    if (motor == LEFT) {
      valid = RCmc->ReadM1VelocityPID(rcAddress, p->P,p->I,p->D,p->qpps);
    } else if (motor == RIGHT) {
      valid = RCmc->ReadM2VelocityPID(rcAddress, p->P,p->I,p->D,p->qpps);
    } else {
      return false;
    }
    return valid;
}

/** updateHardwarePID
*
* New PID settings have been recieved from ROS via a serial command
* update the class variables and execute the hardware PID adjustment
*
* @param  int  pid_args  an array of 4 elements P,I,D,qpps from ROS
* @return bool
*/
bool ROSArdRoboclaw::updateHardwarePID(int *pid_args) {
   leftMotor.P     = pid_args[0];
   rightMotor.P    = pid_args[0];
   leftMotor.D     = pid_args[1];
   rightMotor.D    = pid_args[1];
   leftMotor.I     = pid_args[2];
   rightMotor.I    = pid_args[2];
   leftMotor.qpps  = pid_args[3];
   rightMotor.qpps = pid_args[3];
   bool valid;
   valid  = sendPIDtoRC(LEFT);
   valid &= sendPIDtoRC(RIGHT);
   return valid;
}


/** initMotorController
 *
 * Initialalize the motor controller. Start it up, set the initial PID
 * parameters, and reset the encoders.
 */
bool ROSArdRoboclaw::initMotorController() {
    // set up base values for a default RoboClaw robot.
    hasEncoder            = true;
    hasMotorController    = true;
    softPID               = false;
    hasSensors            = false;
    hasServos             = false;

    RCmc->begin(getcbBaud());
    
    // load the intial settings for the hardware velocity PID done by the
    // roboclaw controller @ 300hz
    bool valid;
    valid =  sendPIDtoRC(LEFT);
    valid &= sendPIDtoRC(RIGHT);
   
    // reset the encoders at the beginning of an initialization
    valid &= resetEncoders();
    
    return valid;
}

/** updatePID
 *
 *  Update the setpoints for the LEFT and RIGHT motor controller. On RoboClaw the Velocity PID
 *  is handled by the motorcontroller board at 300 hz rather than in software on the Arduino.
 *  If a software implemnation is desired, it would go here.
 * @return  void
 */
void ROSArdRoboclaw::updatePID() {
    // reading the encoders isnt' strictly necessary when using hardware PID, but can't hurt.
    // would be used if softPID is enabled.
    readEncoders(leftSetPoint.Encoder, rightSetPoint.Encoder);

    if (softPID == true) {
        if (!isMoving()) {
            /*
             * Reset PIDs once, to prevent startup spikes,
             * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
             * PrevInput is considered a good proxy to detect
             * whether reset has already happened
             */
            if (leftSetPoint.PrevInput != 0 || rightSetPoint.PrevInput != 0) resetPID();
            return;
        }
        // Compute PID update for each motor
        doPID(&(leftSetPoint));
        doPID(&(rightSetPoint));

        /* Set the motor speeds accordingly */
        setMotorSpeeds(leftSetPoint.output, rightSetPoint.output);
    } else {
        // because on the RoboClaw, we're relying on the board to perform the velocity PID
        // we simply feed it new velocity setpoints and let it do it's thing.
        
//TODO  // TargetTicksPerFrame will need to be converted correctly to qpps (quadrature pulses per second)
        // or vice-versa depending on how ROS expects to send commands.
        setMotorSpeeds(leftSetPoint.TargetTicksPerFrame,rightSetPoint.TargetTicksPerFrame);
    }
}

/** doPID
 *
 * With software PID, this would be the routine to calculate the software PID
 * Roboclaw supports hardware Velocity PID, thus we need do nothing here. Software implementation
 * included incase we want to rely upon it for some reason.
 *
 * @param  object   SetPointInfo *p    SetPointInfo contains the PID calculations for software PID
 * @return void
 */
void ROSArdRoboclaw::doPID(SetPointInfo *p) {
    if (getsoftPID() == true) {
        long Perror;
        long output;
        int input;
        
        //Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);
        input = p->Encoder - p->PrevEnc;
        Perror = p->TargetTicksPerFrame - input;
        
        /*
         * Avoid derivative kick and allow tuning changes,
         * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
         * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
         */
        //output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
        // p->PrevErr = Perror;
        output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
        p->PrevEnc = p->Encoder;
        
        output += p->output;
        // Accumulate Integral error *or* Limit output.
        // Stop accumulating when output saturates
        if (output >= getMaxPWM()) {
             output = getMaxPWM();
        } else if (output <= -getMaxPWM()) {
            output = -getMaxPWM();
        } else {
        /*
         * allow turning changes, see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
         */
            p->ITerm += Ki * Perror;
        }
        
        p->output = output;
        p->PrevInput = input;
    } else {
        // hardward PID. Nothing needs to be done here.
    }
}






