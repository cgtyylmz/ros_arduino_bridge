#ifndef ROSARDTYPES_H
#define ROSARDTYPES_H

// Closed Loop Velocity Control PID settings for quadrature encoders
typedef struct {
  float P;
  float I;
  float D;
  uint32_t qpps;
} VPID;

// Closed Loop Position Control PID settings
typedef struct {
  float P;
  float I;
  float D;
  float Imax;
  uint32_t Dead;
  uint32_t Min;
  uint32_t Max;
} PPID;


// PID setpoint info For a Motor. Used for Software PID implmented on Arduino.
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
  * Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input
  int PrevErr;                  // last error

  /*
  * Using integrated term (ITerm) instead of integrated error (Ierror),
  * to allow tuning changes,
  * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  //int Ierror;
  int ITerm;                    //integrated term

  long output;                  // last motor setting
} SetPointInfo;

#endif // ROSARDTYPES_H

