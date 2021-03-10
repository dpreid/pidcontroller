#include "pid.h"

PID::PID(float Kp, float Ki, float Kd, float Ts, float N, float uMin, float uMax) {
  setAll(Kp, Ki, Kd, Ts, N, uMin, uMax);
}

void PID::setAll(float Kp, float Ki, float Kd, float Ts, float N, float uMin, float uMax) {

  reset();

  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
  Ts_ = Ts;
  N_ = N;
  uMin_ = uMin;
  uMax_ = uMax;

  a0 = (1+N*Ts);
  a1 = -(2 + N*Ts);
  a2 = 1;
  b0 = Kp*(1+N*Ts) + Ki*Ts*(1+N*Ts) + Kd*N;
  b1 = -(Kp*(2+N*Ts) + Ki*Ts + 2*Kd*N);
  b2 = Kp + Kd*N;
  ku1 = a1/a0;
  ku2 = a2/a0;
  ke0 = b0/a0;
  ke1 = b1/a0;
  ke2 = b2/a0;

}

void PID::reset(void) { //TODO consider making this private
  e0 = 0;
  e1 = 0;
  e2 = 0; 
  u0 = 0;
  u1 = 0;
  u2 = 0;
}

void PID::setCommand(float command) {
  r = command;
}

float PID::getCommand(void) {
  return r;
}

float PID::getError(void) {
  return e0;
}

float PID::update(float y){

  // shift history
  e2=e1;
  e1=e0;
  u2=u1;
  u1=u0;

  e0=r-y; //compute new error

  // https://en.wikipedia.org/wiki/Integral_windup
  //if ( (e0 * e1) <= 0) { //reset integrator on zero crossing
  //	u1 = 0;
  //	u2 = 0;
	//}
  
  u0 = -ku1*u1 - ku2*u2 + ke0*e0 + ke1*e1 + ke2*e2; //eq(12)

  //if (u0 > uMax_) u0 = uMax_; //limit to plant range
  //if (u0 < uMin_) u0 = uMin_;

  return u0;

}

void PID::setKs(float Kp, float Ki, float Kd) {
  setAll(Kp, Ki, Kd, Ts_, N_, uMin_, uMax_);
}

void PID::setKp(float Kp) {
  setAll(Kp, Ki_, Kd_, Ts_, N_, uMin_, uMax_);
}

void PID::setKi(float Ki) {
  setAll(Kp_, Ki, Kd_, Ts_, N_, uMin_, uMax_);
}

void PID::setKd(float Kd) {
  setAll(Kp_, Ki_, Kd, Ts_, N_, uMin_, uMax_);
}

void PID::setTs(float Ts) {
  setAll(Kp_, Ki_, Kd_, Ts, N_, uMin_, uMax_);
}

void PID::setN(float N) {
  setAll(Kp_, Ki_, Kd_, Ts_, N, uMin_, uMax_);
}

void PID::setLimits(float uMin, float uMax) {
  setAll(Kp_, Ki_, Kd_, Ts_, N_, uMin, uMax);
}

float PID::getKp(void) {
  return Kp_;
}
float PID::getKi(void){
  return Ki_;
}

float PID::getKd(void) {
  return Kd_;
}

float PID::getTs(void){
  return Ts_;
}
float PID::getN(void) {
  return N_;
}

float PID::getUMax(void) {
  return uMax_;
}

float PID::getUMin(void) {
  return uMin_;
}

bool PID::hasZeroHistory(void){
  return (
		  e0 == 0 &&
		  e1 == 0 &&
		  e2 == 0 &&
		  u0 == 0 &&
		  u1 == 0 &&
		  u2 == 0   
		  );
}
