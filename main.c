#include <stdlib.h>
#include <stdio.h>
#include <math.h> // Required for <tinyspline.h>, M_PI, atan2, sin, cos
#include <errno.h> // Error Checking
#include <string.h> // Required for strerror, <crc.h>
#include <arpa/inet.h>
#include <time.h> // srand

#include "tinyspline.h"
#include "crc.h"

#include "CPFrames.h"

#define MaxTextExtent  4096 /* always >= 4096 */
#define MaxCtrlPointsExtent 100 /* always >= 50 */

#define ShoulderPanLinkLength 8.75
// #define ElbowPanLinkLength 8.75
#define ElbowPanLinkLength 9.695
#define PPI 72

// #define ZDrawingPlane 50
#define ZDrawingPlane 0
#define ZRetractPlane 15

#define ZActuatorCalibrationBL 200
#define ZActuatorCalibrationBR 300
#define ZActuatorCalibrationTL 550
#define ZActuatorCalibrationTR 750

#define WorkspaceLength 15.5
#define WorkspaceWidth 9.5

// Returns the value to be added to the computed actuator value to account for the arm plane not being parallel to the paper plane
float actuator_delta(float x, float y) {
  // The two deltas for each direction SHOULD be the same, but we average them in practice.
  float slope_x = ((ZActuatorCalibrationBR - ZActuatorCalibrationBL + ZActuatorCalibrationTR - ZActuatorCalibrationTL) * 0.5) / WorkspaceLength;
  float slope_y = ((ZActuatorCalibrationTL - ZActuatorCalibrationBL + ZActuatorCalibrationTR - ZActuatorCalibrationBR) * 0.5) / WorkspaceWidth;

  // We assume an origin at the bottom left.
  return x * slope_x + y * slope_y + slope_x*(WorkspaceWidth/2.0);
  // return 0;
}

tsRational** spline_to_cartesian(tsBSpline *spline, float increment, size_t *size)
{
  tsRational u;
  tsDeBoorNet net;

  // *size = 1.f/increment + 2;
  *size = 1.f/increment;

  tsRational **cartesian = malloc(sizeof(tsRational *)* (*size));

  size_t i = 0;

  // {
  //   ts_bspline_evaluate(spline, 0.f, &net);
  //   tsRational *result = malloc(sizeof(tsRational) * 3);
  //   result[0] = net.result[0]/PPI - 8.5; // x
  //   result[1] = 15 - net.result[1]/PPI; // y
  //   result[2] = -1; // z
  //   cartesian[i] = result;
  //   printf("%zd, %f, %f, %f\n", i, cartesian[i][0], cartesian[i][1], cartesian[i][2]);
  //   i++;
  // }

  for (u = 0.f; u <= 1.f; u += increment)
  {
    ts_bspline_evaluate(spline, u, &net);

    // Store in Memmory
    tsRational *result = malloc(sizeof(tsRational) * 3);
    result[0] = net.result[0]/PPI - 8.5; // x
    // result[0] = result[0] - 0.25; // TMP Adjustment for Strange Paper
    result[1] = 15 - net.result[1]/PPI; // y
    // result[1] = result[1]-3.0; // y
    result[2] = 0; // z
    cartesian[i] = result;
    // printf("%zd (%03.2f), %f, %f, %f\n", i, u, cartesian[i][0], cartesian[i][1], cartesian[i][2]);
    i++;

  }

  // {
  //   tsRational *result = malloc(sizeof(tsRational) * 3);
  //   result[0] = net.result[0]/PPI - 8.5; // x
  //   result[1] = 15 - net.result[1]/PPI; // y
  //   result[2] = -1; // z
  //   cartesian[i] = result;
  //   printf("%zd, %f, %f, %f\n", i, cartesian[i][0], cartesian[i][1], cartesian[i][2]);
  // }

  // printf("Size: %zd, i: %zd\n", *size, i);

  ts_deboornet_free(&net);

  return cartesian;
}

tsRational** cartesian_to_motor_angles(tsRational **cartesian, size_t size)
{

  float **transformation = malloc(sizeof(float *)* size);

  size_t i;
  for (i = 0; i <= size; i++)
  {
    float *result = malloc(sizeof(float) * 3);
    result[0] = 10;
    result[1] = 20;
    result[2] = 30;
    transformation[i] = result;
  }

  return transformation;

  // tsRational **transformation = malloc(sizeof(tsRational *)* size);

  // size_t i;
  // for (i = 0; i <= size; i++)
  // {
  //   tsRational result[3];
  //   // tsRational *result = malloc(sizeof(tsRational) * 3);
  //   memset(result, 0, sizeof(tsRational) * 3);
  //   // float theta1, theta2, r;

  //   // r = (pow(cartesian[i][0],2)+pow(cartesian[i][1],2)-pow(ShoulderPanLinkLength,2)-pow(ElbowPanLinkLength,2))/(2*ShoulderPanLinkLength*ElbowPanLinkLength);
  //   // theta2 = atan2(sqrt(1-pow(r,2)),r);
  //   // theta1 = atan2(cartesian[i][1], cartesian[i][0]) - atan2(ElbowPanLinkLength*sin(theta2), ShoulderPanLinkLength+ElbowPanLinkLength*cos(theta2));

  //   result[0] = 30;
  //   result[1] = 40;
  //   result[2] = 50;
  //   // // result[0] = roundf(theta1*437.04);
  //   // result[3] = 40;
  //   // // result[1] = roundf(theta2*437.04);
  //   // result[4] = 50;
  //   // result[5] = 60;
  //   // if (cartesian[i][2] == -1){
  //   //   result[2] = ZRetractPlane;
  //   // }else{
  //   //   result[2] = ZDrawingPlane+actuator_delta(cartesian[i][0], cartesian[i][1]);
  //   // }

  //   transformation[i] = result;

  //   // printf("R%zd, %f, %f, %f, %f, %f, %f\n", i, result[0], result[1], result[2], result[3], result[4], result[5]);
  //   // printf("R%zd, %f, %f, %f\n", i, result[0], result[1], result[2]);
  //   printf("R%zd, %f, %f, %f\n", i, transformation[i][0], transformation[i][1], transformation[i][2]);

  // }

  // size_t q;
  // for (q = 0; q <= size; q++)
  // {
  //   tsRational *result = transformation[q];
  //   printf("TQ%zd, %f, %f, %f\n", q, result[0], result[1], result[2]);
  // }

  // // printf("Recieved Size: %zd, i: %zd\n", size, i);
  
  // return transformation;
}

CPFrameVersion02 *motor_angles_to_packet(tsRational **transformation, size_t size)
{
  CPFrameVersion02 *packets = malloc(sizeof(tsRational *)* size);
  size_t i;
  for (i = 0; i <= size; i++)
  {
    tsRational *result = transformation[i];
    short theta1, theta2, d3;
    theta1 = floor(result[0]); theta2 = floor(result[1]); d3 = floor(result[2]);
    CPFrameVersion02 frame = {StartFrameDelimiter, CPV02_VERSION, 0, theta1, theta2, d3, 0, EndOfFrame};
    frame.CRC = crcFast((unsigned char *) &frame, CPV02_SIZE-3);
    packets[i] = frame;

    printf("P%zd, <%f> %hd, <%f> %hd, <%f> %hd\n", i, result[0], theta1, result[1], theta2, result[2], d3);
  }

  return packets;
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

int motion_planning_packets(const char *curves_file, const char *packets_buffer_file)
{
  FILE* file = fopen(curves_file, "r");
  if(file == NULL)
  {
    fprintf(stderr,"File Null Error <%s>: %s\n", curves_file, strerror(errno));
    exit(EXIT_FAILURE);
  }

  FILE *packets_buffer = fopen(packets_buffer_file, "wb+");
  if(file == NULL)
  {
    fprintf(stderr,"File Null Error <%s>: %s\n", packets_buffer_file, strerror(errno));
    exit(EXIT_FAILURE);
  }

  crcInit();
  srand(time(NULL));   // should only be called once
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
    
    if (feof(file) || buffer[len-1] == '\n')
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
        fprintf(stderr,"Error %s: Improperly Defined Bezier Curve\n", curves_file);
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
      cartesian = spline_to_cartesian(&spline, 0.25f, &size);
      size = 4;
      transformation = cartesian_to_motor_angles(cartesian, size);

      size_t q;
      for (q = 0; q <= size; q++)
      {
        printf("TT%zd, %f, %f, %f\n", q, transformation[q][0], transformation[q][1], transformation[q][2]);
      }

      // // Form Packet
      // CPFrameVersion02 *packets = motor_angles_to_packet(transformation, size);

      // for (i = 0; i < size; i++)
      // {
      //   size_t bytes_written = fwrite(&packets[i], sizeof(CPFrameVersion02), 1, packets_buffer);
      //   if (bytes_written != 1)
      //   {
      //     fprintf(stderr,"Error: File Write Operation\n");
      //     exit(EXIT_FAILURE);
      //   }
      // }

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
    fprintf(stderr,"File EOF Error %s: %s\n", packets_buffer_file, strerror(errno));
    exit(EXIT_FAILURE);
  }

  // Clean Up
  fclose(file);
  fclose(packets_buffer);
  free(ret);
  return EXIT_SUCCESS;
}

int main(int argc, char** argv)
{

  if (argc != 3)
  {
    fprintf(stdout,"Usage: %s <curves file> <packets file>\n", argv[0]);
    exit(EXIT_FAILURE);
  }

  return motion_planning_packets(argv[1], argv[2]);
}