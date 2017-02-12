#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "tinyspline.h"

// float** 
tsRational** spline_to_cartesian(tsBSpline *spline, float increment, size_t *size)
{
  tsRational u;
  tsDeBoorNet net;

  *size = 1.f/increment;

  tsRational **cartesian = malloc(sizeof(tsRational *)* (*size));

  size_t i = 0;
  for (u = 0.f; u < 1.f; u += increment)
  {
    ts_bspline_evaluate(spline, u, &net);

    // Store in Memmory
    tsRational *result = malloc(sizeof(tsRational) * 2);
    result[0] = net.result[0]; result[1] = net.result[1];
    cartesian[i] = result;
    i++;
  }

  ts_deboornet_free(&net);

  return cartesian;
}

tsRational** cartesian_to_motor_angles(tsRational **cartesian, size_t size)
{

  tsRational **transformation = malloc(sizeof(tsRational *)* size);

  size_t i;
  for (i = 0; i < size; i++)
  {
    tsRational* result = cartesian[i];
    printf("x: %f, y: %f\n", result[0], result[1]);
  }
  
  return transformation;
}

tsRational** destroy_cartesian(tsRational **cartesian, size_t size)
{
  size_t i;
  for (i = 0; i < size; i++)
  {
    free((tsRational *) cartesian[i]);
  }
  free(cartesian);

  return NULL;
}


int main(int argc, char** argv)
{

  tsBSpline spline;
  ts_bspline_new(
    3,      /* degree of spline */
    2,      /* dimension of each point */
    7,      /* number of control points */
    TS_CLAMPED, /* used to hit first and last control point */
    &spline /* the spline to setup */
  );

  /* Setup control points. */
  spline.ctrlp[0] = -1.75;
  spline.ctrlp[1] = -1.0;

  spline.ctrlp[2] = -1.5;
  spline.ctrlp[3] = -0.5;

  spline.ctrlp[4] = -1.5;
  spline.ctrlp[5] = 0.0;

  spline.ctrlp[6] = -1.25;
  spline.ctrlp[7] = 0.5;

  spline.ctrlp[8] = -0.75;
  spline.ctrlp[9] = 0.75;

  spline.ctrlp[10] = 0.0;
  spline.ctrlp[11] = 0.5;

  spline.ctrlp[12] = 0.5;
  spline.ctrlp[13] = 0.0;

  size_t size;
  tsRational **cartesian, **transformation;
  cartesian = spline_to_cartesian(&spline, 0.1f, &size);
  transformation = cartesian_to_motor_angles(cartesian, size);

  // Clean Up
  cartesian = destroy_cartesian(cartesian, size);
  return 0;
}
