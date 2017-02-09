#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "tinyspline.h"

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


  tsRational u;
  tsDeBoorNet net;
  for (u = 0.f; u < 1.f; u += 0.1f)
  {
    ts_bspline_evaluate(&spline, u, &net);
    printf("x: %f, y: %f\n", net.result[0], net.result[1]);
  }


  ts_deboornet_free(&net);

  return 0; 
}
