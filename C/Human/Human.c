/*
 * Human.c
 *
 *  Created on: Aug 8, 2012
 *      Author: madisob
 */

#include <stdio.h>

#include <math.h>

#include "Human.h"

#include "quaternion.h"
#include "quatsubtraction.h"
#include "quatadd.h"
#include "quatdot.h"
#include "quatnormsquared.h"
#include "quatnorm.h"

int Power[9];

//! Distance metric for the Human Body
/*!
 Takes two poses and finds a vector of weighted distance between the two.

 The difference if of the form V[i] = ||P1[i] - P2[i]|| * W[i] ,  i = [0, POSE_DIMENIONS])
 where || . || denotes quaternion norm

 \param P1	Pose one
 \param P2	Pose two
 \param W 	Weight vector
 \param V	The error on each dimension
 \returns	Weighted difference between P1 and P2
 */
void humanError(const POSE P1, const POSE P2, const real_num* W, real_num* V)
{
	int i;
	for(i = 0 ; i < POSE_DIMENSION ; i++)
	{

		//printf("%0.5f %0.5f %0.5f %0.5f \n", P1[i].w, P1[i].x, P1[i].y, P1[i].z);
		//printf("%0.5f %0.5f %0.5f %0.5f \n\n", P2[i].w, P2[i].x, P2[i].y, P2[i].z);
		quaternion arith;
		quatsubtraction(&P1[i], &P2[i], &arith);

		//printf("%0.20f\n", P1[i].w);

		//NUM QNorm = sqrt(quatnormsquared(&arith));
		Q_real_num QNorm;
		quatnorm(&arith, &QNorm);
		//quatnormsquared(&arith, &QNorm);

		real_num A = (real_num)QNorm * W[i];
		
		//if(A > 2.2)
			//V[i] = 100000000;
		//else
			V[i] = A*A;
		//else
			//V[i] = pow(A, A+2);
		//else
			//V[i] = A;
		
		//if(V[i] < 1.0)
			//V[i] = 0.0;
		
		//printf("V: %f\n", A);
	}
}



void humanAddError(const real_num* E1, const real_num* E2, real_num* EAdd)
{
	int i;
	for(i = 0 ; i < POSE_DIMENSION ; i++)
	{
		//if(E2[i] < 1.0)
			//EAdd[i] = E1[i];
		//else
		//	EAdd[i] = E1[i] + E2[i];
		EAdd[i] = E1[i] + E2[i];
	}
}

void humanDivideError(const real_num* EQ, const real_num D, real_num* EDivide)
{
	int i;
	for(i = 0 ; i < POSE_DIMENSION ; i++)
	{
		EDivide[i] = EQ[i] / D;	
	}
}


// Sum Each Element
real_num humanRealError(const real_num* E)
{
	int i;
	real_num V = 0;
	for(i = 0 ; i < POSE_DIMENSION ; i++)
	{
		if(E[i] > V)
			V = E[i];
		//V = V + E[i];
		
	}
	return V;
}


