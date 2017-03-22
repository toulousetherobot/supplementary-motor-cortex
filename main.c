#include <stdlib.h>
#include <stdio.h>
#include <math.h> // Required for tinyspline
#include <errno.h> // Error Checking
#include <string.h> // Required for strerror()

#include "tinyspline.h"

#define MaxTextExtent  4096 /* always >= 4096 */
#define MaxCtrlPointsExtent 100 /* always >= 50 */

#define ShoulderPanLinkLength 8.75
#define ElbowPanLinkLength 8.75
#define PPI 72 

tsRational** spline_to_cartesian(tsBSpline *spline, float increment, size_t *size)
{
  tsRational u;
  tsDeBoorNet net;

  *size = 1.f/increment;

  tsRational **cartesian = malloc(sizeof(tsRational *)* (*size));

  size_t i = 0;
  for (u = 0.f; u <= 1.f; u += increment)
  {
    ts_bspline_evaluate(spline, u, &net);

    // Store in Memmory
    tsRational *result = malloc(sizeof(tsRational) * 2);
    result[0] = net.result[0]/PPI - 8.5; // x
    result[1] = 15 - net.result[1]/PPI; // y
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

  int found_tool = 0;
  int tool_number = 0;
  tsRational *points;
  size_t count = -1;
  char *ret = NULL;
  size_t full_length = 0;

  while (fgets(buffer, sizeof(buffer), file))
  {
    /* note that fgets don't strip the terminating \n, checking its
      presence would allow to handle lines longer that sizeof(buffer) */

    size_t len = strlen(buffer);
    if (SIZE_MAX - len - 1 < full_length)
    {
      break;
    }
    char *r_temp = realloc(ret, full_length + len + 1);
    if (r_temp == NULL)
    {
      break;
    }
    ret = r_temp;
    strcpy(ret + full_length, buffer); /* concatenate */
    full_length += len;
    
    if (feof(stdin) || buffer[len-1] == '\n')
    {
      // Complete Toolpath Found
      char *new_line_char = strchr(ret, '\n');
      if (new_line_char != NULL || feof(file))
      {
        // Truncate Buffer
        if (new_line_char != NULL)
          *new_line_char = '\0';

        // Reset Toolpath Variables
        found_tool = 0;
        points = malloc(sizeof(tsRational) * MaxCtrlPointsExtent);
        count = -1;
      }

      // Parsing Tool Number
      char *pt;
      if (!found_tool)
      {
        pt = strtok (ret, ";");
        if (pt != NULL)
        {
          found_tool = 1;
          tool_number = atoi(pt);
        }
      }

      // Parsing Control Points
      if (found_tool)
        pt = strtok (NULL, ",");
      else
        pt = strtok (ret, ",");
      while (pt != NULL) 
      {
        count++;
        points[count] = atof(pt);
        pt = strtok (NULL, ",");
      }

      // Checking Compliance for Dimensions of Control Points
      if (count % 2 == 0)
      {
        fprintf(stderr,"Error %s: Improperly Defined Bezier Curve\n", argv[0]);
        exit(EXIT_FAILURE);
      }

      // Building Spline
      tsBSpline spline;
      ts_bspline_new(
        (count/3 == 1) ? 1 : 3,      /* degree of spline */
        2,      /* dimension of each point */
        (count+1)/2,      /* number of control points */
        TS_CLAMPED, /* used to hit first and last control point */
        &spline /* the spline to setup */
      );

      // Setup Control Points
      size_t i;
      for (i = 0; i < count+1; i++)
      {
        spline.ctrlp[i] = points[i];
      }

      // Transform Spline to Inverse Kinematics
      size_t size;
      tsRational **cartesian, **transformation;
      cartesian = spline_to_cartesian(&spline, 0.1f, &size);
      transformation = cartesian_to_motor_angles(cartesian, size);

      // Clean Up
      cartesian = destroy_cartesian(cartesian, size);
      transformation = destroy_cartesian(transformation, size);
      free(points);
      free(ret); ret = (char *) NULL;
      full_length = 0;
    }
  }

  if (!feof(file))
  {
    fprintf(stderr,"File EOF Error <%s>: %s\n", argv[1], strerror(errno));
    exit(EXIT_FAILURE);
  }

  fclose(file);
  free(ret);
  return EXIT_SUCCESS;
}