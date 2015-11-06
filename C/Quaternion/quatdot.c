/*
 * quatdot.c
 *
 *  Created on: Sep 11, 2012
 *      Author: madisob
 */

#include "quatdot.h"

//!  Take the dot product of a quaterion
/*
 \param Q1 Left side of dot product
 \param Q2 Right side of dot product
 \param Qdot result of dot(Q1, Q2)
 */
void quatdot(const quaternion* Q1, const quaternion* Q2, Q_real_num* Qdot)
{
	*Qdot = Q1->x*Q2->x + Q1->y*Q2->y + Q1->z*Q2->z;
}
