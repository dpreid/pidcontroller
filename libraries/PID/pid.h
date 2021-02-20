#ifndef PIDLIB_PID_H_
#define PIDLIB_PID_H_
#endif

//Calculates the control signal to apply to a plant,
//  given an error, using PID controller in discrete
//  Z-transform format, according to difference equation
//  Eq. 12 at
//
//  https://www.scilab.org/discrete-time-pid-controller-implementation
//
//  Note that this controller has memory (history) so needs
//  resetting when parameters change

class PID {

  double e2, e1, e0, u2, u1, u0;  // variables used in PID computation
  double a0, a1, a2, b0, b1, b2;  // coefficients 
  double ku1, ku2, ke0, ke1, ke2; // coefficients 
  
  double r; // current command (desired plant output)

  // These parameters are user-adjustable
  // Derived parameters in difference equation must be recalculated
  // when these are changed.

  double _Kp;    // proportional gain
  double _Ki;    // integral gain
  double _Kd;    //  derivative gain
  double _N;     // filter coefficients e.g. 20
  double _Ts;   // This must match actual sampling time PID, in seconds e.g. 0.02 for 50Hz
  double _uMin, _uMax; //plant limits
  
  void reset(void); // set* commands will call this whenever any changes warrant it
  
 public:
  PID(float Kp, float Ki, float Kd, float Ts, float N, float uMin, float uMax);
  void setAll(float Kp, float Ki, float Kd, float Ts, float N, float uMin, float uMax);
  void setKs(float Kp, float Ki, float Kd);
  void setKp(float Kp);
  void setKi(float Ki);
  void setKd(float Kd);
  void setTs(float Ts);
  void setN(float N);
  void setLimits(float umin, float umax);
  float getKp(void);
  float getKi(void);
  float getKd(void);
  float getTs(void);
  float getN(void);
  float getUMax(void);
  float getUMin(void);
  bool hasZeroHistory(void); //helper for testing
  void setCommand(float command); //r is desired plant output
  float update(float y);     //y is actual plant output
  
};


