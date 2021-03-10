#include "dcmotor.h"

Driver::Driver(float *x, float *y, unsigned int size) {

  curve0 = {
				size,      /* Number of data points */
				x, /* Array of x-coordinates */
				y  /* Array of y-coordinates */
  };

  curve1 = curve0; //simplify logic elsewhere by always having a meaningful curve1

}

/* 

 condition  s < t  |  useSecondCurveBelowThreshold  | curve
-------------------|-----------------------------------|-------
 above      False  |              False                |  curve1
 above      False  |              True                 |  curve0 
 below      True   |              False                |  curve0
 below      True   |              True                 |  curve1
        */


float Driver::drive(float primary, float secondary) {

  if (primary == 0.0) {
	primary = epsilon;
  }
  float val = 0;
  
  if (ABS(secondary) <
	  ABS(threshold) == useSecondCurveBelowThreshold ) {
	val = interpolate_table_1d(&curve1, primary);
  } else {
	val = interpolate_table_1d(&curve0, primary);
	if (primary > primaryOffsetThreshold) {
	  val += primaryOffsetPos;
	}
	if (primary < -primaryOffsetThreshold) {
	  val += primaryOffsetNeg;
	}
  }
  if (ABS(val) < epsilon) {
	  val = 0.0;
	}  
	return val;
}

void Driver::updatePrimaryCurve(float *x, float *y, unsigned int size) {
  
  curve0 = {
				size,      /* Number of data points */
				x, /* Array of x-coordinates */
				y  /* Array of y-coordinates */
  };
  
}

void Driver::addSecondCurve(float *x, float *y, unsigned int size) {
  
  curve1 = {
				size,      /* Number of data points */
				x, /* Array of x-coordinates */
				y  /* Array of y-coordinates */
  };
  
}

/**
* Returns the interpolated y-value.
* Saturates to y0 or y1 if x outside interval [x0, x1].
*/
float Driver::interpolate_segment(float x0, float y0, float x1, float y1, float x)
{
    float t;

    if (x <= x0) { return y0; }
    if (x >= x1) { return y1; }

    t =  (x-x0);
    t /= (x1-x0);

    return y0 + t*(y1-y0);
}

/* 1D Table lookup with interpolation */
float Driver::interpolate_table_1d(struct table_1d *table, float x)
{
    unsigned int segment;

    /* Check input bounds and saturate if out-of-bounds */
    if (x > (table->x_values[table->x_length-1])) {
       /* x-value too large, saturate to max y-value */
        return table->y_values[table->x_length-1];
    }
    else if (x < (table->x_values[0])) {
       /* x-value too small, saturate to min y-value */
        return table->y_values[0];
    }

    /* Find the segment that holds x */
    for (segment = 0; segment<(table->x_length-1); segment++)
    {
        if ((table->x_values[segment]   <= x) &&
            (table->x_values[segment+1] >= x))
        {
            /* Found the correct segment */
            /* Interpolate */
            return interpolate_segment(table->x_values[segment],   /* x0 */
                                       table->y_values[segment],   /* y0 */
                                       table->x_values[segment+1], /* x1 */
                                       table->y_values[segment+1], /* y1 */
                                       x);                         /* x  */
        }
    }

    /* Something with the data was wrong if we get here */
    /* Saturate to the max value */
    return table->y_values[table->x_length-1];
}

