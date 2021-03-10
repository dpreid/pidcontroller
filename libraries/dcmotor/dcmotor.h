#ifndef DCMOTOR_DCMOTOR_H_
#define DCMOTOR_DCMOTOR_H_
#endif

// The Driver class implements mapping with one, or two, N-segment interpolation curves.
// The second curve, if supplied, is applied depending on whether the secondary measurement supplied
//   to the driver is above or below the threshold. For example:
//   yourDriver.threshold = 0.5;
//   yourDriver.useSecondCurveBelowThreshold = true;
// 
// For a secondary value of 0.2, the second curve is applied. For a secondary value of 0.6,
//   The first curve is applied.
//

#define ABS(a) ((a) < 0 ? -(a) : (a))

class Driver {
  
  /* Structure definition */
  struct table_1d {
    unsigned int x_length;
    float *x_values;
    float *y_values;
  };

  table_1d curve0, curve1;
  const float epsilon = 1e-12;
  float primaryNeg, primaryPos;
  float interpolate_table_1d(struct table_1d *table, float x);
  float interpolate_segment(float x0, float y0, float x1, float y1, float x);
  
 public:
  float threshold = 0; //set to zero to avoid using second curve compensation
  float primaryOffsetThreshold = 0; //set zero to use primary offset all the time
  bool useSecondCurveBelowThreshold = true; //set to false if you want the second curve when secondary ABOVE threshold
  float primaryOffsetNeg = 0; //added to primary curve for  x < 0 
  float primaryOffsetPos = 0; // added to primary curve for x > 0 
  Driver(float *x, float *y, unsigned int size);
  void updatePrimaryCurve(float *x, float *y, unsigned int size);
  void addSecondCurve(float *x, float *y, unsigned int size);
  float drive(float primary, float secondary);
};

