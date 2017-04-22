#include <stdlib.h>
#include <stdio.h>
#include <string.h>

float** cartesian_to_motor_angles(size_t size)
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
}

int main(int argc, char** argv)
{

	size_t size = 5;

	float** transformation = cartesian_to_motor_angles(size);

	size_t q;
	for (q = 0; q <= size; q++)
	{
		float *result = transformation[q];
		printf("TT%zd, %f, %f, %f\n", q, result[0], result[1], result[2]);
	}

}