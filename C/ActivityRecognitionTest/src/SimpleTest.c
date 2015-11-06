/*
 * SimpleTest.c
 *
 *  Created on: Aug 15, 2012
 *      Author: madisob
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MatchPatternToObserved.h"
#include "Pattern.h"
#include "Sequence.h"

#include "quatnorm.h"
#include "quatsubtraction.h"

#define DIMENSIONS 1

/* this is the definition of the anonymous function */
#define lambda(l_ret_type, l_arguments, l_body)         \
  ({                                                    \
	l_ret_type l_anonymous_functions_name l_arguments   \
	  l_body                                            \
	&l_anonymous_functions_name;                        \
  })

static const int PATTERNLENGTH = 4;
static const int OBSERVEDLENGTH = 4;

float ErrorFunction(const CHARACTER C1, const CHARACTER C2, const float* W);

int SimpleTest()
{
	int error_return = 0;
	int i;


	//1 dimension and the weight is 1
	float W[DIMENSIONS] = {1};

	//Pattern Characters
	int** PC = (malloc(sizeof(int*) * PATTERNLENGTH));
	for(i = 0 ; i < PATTERNLENGTH ; i++)
	{
		PC[i] = malloc(sizeof(int) * DIMENSIONS);
	}
	PC[0][0] = 1;
	PC[1][0] = 3;
	PC[2][0] = 5;
	PC[3][0] = 7;

	float min[4] = {1,1,1,1};
	float max[4] = {1,1,1,1};

	//Text String
	int** TS = (malloc(sizeof(int*) * PATTERNLENGTH));
	for(i = 0 ; i < PATTERNLENGTH ; i++)
	{
		TS[i] = malloc(sizeof(int) * DIMENSIONS);
	}
	TS[0][0] = 1;
	TS[1][0] = 3;
	TS[2][0] = 5;
	TS[3][0] = 7;

	//Construct Activity
	Pattern A;
	A.N = PATTERNLENGTH;
	A.sample_rate = 1.0;
	A.C = (CHARACTER*)PC;
	A.min = min;
	A.max = max;

	//Construct Observed
	Sequence O;
	O.N = OBSERVEDLENGTH;
	O.sample_rate = 1.0;
	O.C = (CHARACTER*)TS;

	//Malloc space to operate on.
	real_num** Space1 = malloc(sizeof(real_num*) * PATTERNLENGTH);
	int_num** Space2 = malloc(sizeof(int_num*) * PATTERNLENGTH);
	for(i = 0 ; i < PATTERNLENGTH ; i++)
	{
		Space1[i] = malloc(sizeof(real_num) * OBSERVEDLENGTH);
		Space2[i] = malloc(sizeof(int_num) * OBSERVEDLENGTH);
	}

	//Set up the error function
	real_num (*Err)(const CHARACTER, const CHARACTER) = lambda(real_num, (const CHARACTER C1, const CHARACTER C2), { return ErrorFunction(C1, C2, W); });

	float Score = matchPatternToObserved(&A, &O, 0, Err, Space1, Space2, 0);
	if(Score != 0.0)
	{
		printf("  The score for this test does not equal Zero!\n");
		printf("  Its: %f", Score);
		error_return = 1;
	}
	else
	{
		printf("  Success!\n");
	}

	return error_return;
}

//! Error function for this example
/*!
 \param C1			Character 1
 \param C2			Character 2
 \param W			Set of Weights
 \return 			Weighted distance from C1 to C2
 \sa Pattern CHARACTER
 */
float ErrorFunction(const CHARACTER C1, const CHARACTER C2, const float* W)
{
	printf("%d - %d\n", ((int*)C1)[0], ((int*)C2)[0]);
	return abs(((int*)C1)[0] - ((int*)C2)[0]);
}
