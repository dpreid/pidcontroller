#include "rotaryPlant.h"

RotaryPlant::RotaryPlant(int pulsesPerRevolution, float lpf, float timestepSeconds) {
  ppr = pulsesPerRevolution;
  Ts = timestepSeconds;
  // low pass filter coefficient for velocity
  // typically
  if (lpf > 1) lpf = 1;
  if (lpf < 0) lpf = 0;
  a = lpf;

  pMax = (ppr / 2);
  pMin = (ppr / 2) * -1;
}

long RotaryPlant::wrap(long raw) {

  if ( raw <= pMax && raw >= pMin) {
	return raw;
  }

  long residue = raw % (ppr);
  
  if ( residue <= pMax && residue >= pMin) {
	return residue;
  }
  
  residue = residue % ( ppr / 2);
  
  if (residue >= 0 ) return pMin + residue;
  return pMax + residue;
}

float RotaryPlant::fractionalPosition(long p) {
  return (float) wrap(p) / (float)ppr;
}

float RotaryPlant::fractionalDisplacement(long p) {
  return (float) p / (float)ppr;
}

void RotaryPlant::setLPF(float lpf) {
  a = lpf;
}


void RotaryPlant::initialise(long position) {
  p0 = position;
  v0 = 0;
  fv0 = 0;
}

void RotaryPlant::sample(long position) {

  p1 = p0;
  v1 = v0;
  fv1 = fv0;
  samples++;
  if (samples > 2) {
	velocityValid = true;
  } else {
	v1 = 0;
	fv1 = 0;
  }
  p0 = position;

  /******* velocity ******/
  
  // use raw position to avoid mistakes due to wrapping
  long dplong  = p0 - p1;
  float dp = (float)dplong / (float)ppr; // displacement unit is fraction of a revolution 

  v0 = dp / Ts; //float division faster than longs (3byte div and one byte subtraction for floats  vs 4byte div for long) 
  
  //https://dsp.stackexchange.com/questions/60277/is-the-typical-implementation-of-low-pass-filter-in-c-code-actually-not-a-typica
  fv0 = ((1 - a) * fv1) + (a * (v0 + v1) / 2);
}

float RotaryPlant::getPosition(void) { // position in units of fraction of a revolution
  return fractionalPosition(p0); //expected range -0.5 <= pos < 0.5
}

float RotaryPlant::getDisplacement(void) { // position in units of fraction of a revolution
  return fractionalDisplacement(p0); //expected range -0.5 <= pos < 0.5
}

float RotaryPlant::getVelocity(void) { // velocity in units of revolutions per second
  //do not move velocity calculation here - it must be done on every sample because it has a filter
  return velocityValid ? fv0 : 0;
}



