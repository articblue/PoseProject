/*
 * Human.h
 *
 *  Created on: Aug 8, 2012
 *      Author: madisob
 */

#ifndef HUMAN_H_
#define HUMAN_H_

#include "Constants.h"
#include "quaternion.h"

//! Dimensions of a pose (N)
static const int POSE_DIMENSION = 9;

//! Space that the human body operates on
typedef quaternion SPACE;

//! The pose is an array of the space (SPACE^N)
typedef SPACE* POSE;

//! Human Segment Enum.
/*! Enum of all 11 human dimensions. UNKOWN is used when it is not known what segment, which means something is wrong
 */
typedef enum _HUMAN_DIM
{
	BODY_X = 0,
	BODY_Y,
	BODY_Z,
	LEFT_UPPER_ARM,
	LEFT_FORE_ARM,
	RIGHT_UPPER_ARM,
	RIGHT_FORE_ARM,
	LEFT_THIGH,
	LEFT_SHIN,
	RIGHT_THIGH,
	RIGHT_SHIN,
	UNKOWN

} HUMAN_SEGMENTS;


//! Distance metric for the Human Body
/*!
 Takes two poses and finds a vector of weighted distance between the two.

 The difference if of the form E[i] = ||P1[i] - P2[i]|| * W[i] ,  i = [0, POSE_DIMENIONS])
 where || . || denotes quaternion norm

 \param P1	Pose one
 \param P2	Pose two
 \param W 	Weight vector
 \param V	The error on each dimension
 \returns	Weighted difference between P1 and P2
 */
void humanError(const POSE P1, const POSE P2, const real_num* W, real_num* E);


void humanAddError(const real_num* E1, const real_num* E2, real_num* EAdd);

void humanDivideError(const real_num* EQ, const real_num D, real_num* EDivide);

real_num humanRealError(const real_num* E);

#endif /* HUMAN_H_ */
