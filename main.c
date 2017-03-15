#include <stdlib.h>
#include <stdio.h>
#include <math.h> // Required for tinyspline
#include <errno.h> // Error Checking
#include <string.h> // Required for strerror()

#include "tinyspline.h"

#define MaxTextExtent  4096 /* always >= 4096 */
#define MaxCtrlPointsExtent 100 /* always >= 50 */

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

  if (argc != 2)
  {
    fprintf(stdout,"Usage: %s curves\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  FILE* file = fopen(argv[1], "r");
  if(file == NULL)
  {
    fprintf(stderr,"File Null Error <%s>: %s\n", argv[1], strerror(errno));
    exit(EXIT_FAILURE);
  }

  char buffer[MaxTextExtent];

  while (fgets(buffer, sizeof(buffer), file))
  {
    /* note that fgets don't strip the terminating \n, checking its
      presence would allow to handle lines longer that sizeof(buffer) */

    char *pt;
    pt = strtok (buffer, ";");
    // printf("Tool No: '%d'\n", atoi(pt));

    tsRational *points = malloc(sizeof(tsRational) * MaxCtrlPointsExtent);
    size_t count = -1;

    pt = strtok (NULL, ",");
    while (pt != NULL) {
      count++;
      points[count] = atof(pt);
      pt = strtok (NULL, ",");
    }

    if (count % 2 == 0)
    {
      fprintf(stderr,"Error %s: Improperly Defined Bezier Curve\n", argv[0]);
      exit(EXIT_FAILURE);
    }

    tsBSpline spline;
    ts_bspline_new(
      1,      /* degree of spline */
      2,      /* dimension of each point */
      (count+1)/2,      /* number of control points */
      TS_CLAMPED, /* used to hit first and last control point */
      &spline /* the spline to setup */
    );

    /* Setup control points. */
    size_t i;
    for (i = 0; i < count+1; i++)
    {
      spline.ctrlp[i] = points[i];
    }

    size_t size;
    tsRational **cartesian, **transformation;
    cartesian = spline_to_cartesian(&spline, 0.1f, &size);
    transformation = cartesian_to_motor_angles(cartesian, size);

    // Clean Up
    cartesian = destroy_cartesian(cartesian, size);

    free(points);
  }

  if (!feof(file))
  {
    fprintf(stderr,"File EOF Error <%s>: %s\n", argv[1], strerror(errno));
    exit(EXIT_FAILURE);
  }

  fclose(file);
  return EXIT_SUCCESS;
}