/*
 * quatadd.c
 *
 *  Created on: Sep 11, 2012
 *      Author: madisob
 */

#include "quatadd.h"

//!  Adds two quaternions
/*
 \param Q1 Left side of addition
 \param Q2 Right side of addition
 \param Qresult Result of Q1 + Q2
 */
void quatAdd(const quaternion* Q1, const quaternion* Q2, quaternion* Qresult)
{
	Qresult->w = Q1->w + Q2->w;
	Qresult->x = Q1->x + Q2->x;
	Qresult->y = Q1->y + Q2->y;
	Qresult->z = Q1->z + Q2->z;
}
