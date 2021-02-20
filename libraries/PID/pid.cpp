#include "pid.h"

PID::PID(float Kp, float Ki, float Kd, float Ts, float N, float uMin, float uMax) {
  setAll(Kp, Ki, Kd, Ts, N, uMin, uMax);
}

void PID::setAll(float Kp, float Ki, float Kd, float Ts, float N, float uMin, float uMax) {

  reset();

  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _Ts = Ts;
  _N = N;
  _uMin = uMin;
  _uMax = uMax;

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

float PID::update(float y){

  // shift history
  e2=e1;
  e1=e0;
  u2=u1;
  u1=u0;

  e0=r-y; //compute new error

  // https://en.wikipedia.org/wiki/Integral_windup
  if ( (e0 * e1) <= 0) { //reset integrator on zero crossing
	u1 = 0;
	u2 = 0;
  }
  
  u0 = -ku1*u1 - ku2*u2 + ke0*e0 + ke1*e1 + ke2*e2; //eq(12)

  if (u0 > _uMax) u0 = _uMax; //limit to plant range
  if (u0 < _uMin) u0 = _uMin;

  return u0;

}

void PID::setKs(float Kp, float Ki, float Kd) {
  setAll(Kp, Ki, Kd, _Ts, _N, _uMin, _uMax);
}

void PID::setKp(float Kp) {
  setAll(Kp, _Ki, _Kd, _Ts, _N, _uMin, _uMax);
}

void PID::setKi(float Ki) {
  setAll(_Kp, Ki, _Kd, _Ts, _N, _uMin, _uMax);
}

void PID::setKd(float Kd) {
  setAll(_Kp, _Ki, Kd, _Ts, _N, _uMin, _uMax);
}

void PID::setTs(float Ts) {
  setAll(_Kp, _Ki, _Kd, Ts, _N, _uMin, _uMax);
}

void PID::setN(float N) {
  setAll(_Kp, _Ki, _Kd, _Ts, N, _uMin, _uMax);
}

void PID::setLimits(float uMin, float uMax) {
  setAll(_Kp, _Ki, _Kd, _Ts, _N, uMin, uMax);
}

float PID::getKp(void) {
  return _Kp;
}
float PID::getKi(void){
  return _Ki;
}

float PID::getKd(void) {
  return _Kd;
}

float PID::getTs(void){
  return _Ts;
}
float PID::getN(void) {
  return _N;
}

float PID::getUMax(void) {
  return _uMax;
}

float PID::getUMin(void) {
  return _uMin;
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
