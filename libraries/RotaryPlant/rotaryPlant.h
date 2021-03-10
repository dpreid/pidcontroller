#ifndef ROTARYPLANT_ROTARYPLANT_H_
#define ROTARYPLANT_ROTARYPLANT_H_
#endif

//Calculates the position and velocity given the
// current count and timestamp from a rotary encoder
//
// ppr: pulses per revolution from your encoder
// 
// lpf: low pass filter coefficient. Set to 1 to
//      have unfiltered velocites. A typical filter
//      value is lpf = 0.1
//
// timeToSeconds: set to 1e-6 if times are in microseconds
//
// Once the plant is instantiated, prime it with the current
// time and encoder count using initialise().
// call sample() in the interrupt routine with the
// latest encoder count and time in e.g. microseconds.
// Then the position and velocity obtained at that time
// can be retrieved any time until the next call to sample()
// by using the getters getPosition(), and getVelocity().

class RotaryPlant{

  float a; //filter coefficient velocity
  float fv0, fv1; //filtered velocity
  long p0, p1;
  int pMax, pMin;
  int ppr; //pulses per revolution expected (depends on if pins are interrupts)
  long samples = 0;
  float Ts;
  float v0, v1; //raw velocity 
  bool velocityValid = false;
  


public:
  RotaryPlant(int pulsesPerRevolution, float lpf, float timestepSeconds);
  long wrap(long raw); // internal function exposed for testing
  float fractionalPosition(long raw); // internal function exposed for testing
  float fractionalDisplacement(long raw); // internal function exposed for testing
  void initialise(long position);
  void sample(long position); // call this on an interrupt
  float getPosition(void); // call when last sampled position is required (bounded to +/-0.5 rev)
  float getDisplacement(void); // call when last sampled unwrapped position is required (unbounded)
  float getVelocity(void); // call when last sampled velocity is required
  void setLPF(float lpf);
};

