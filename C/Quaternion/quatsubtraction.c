/*
 * quatsubtraction.c
 *
 *  Created on: Jul 11, 2012
 *      Author: madisob
 *
 *  Subtracts two quaternions
 */
#include "quatsubtraction.h"

//!  Subtract two quaternions
/*
 \param Q1 Left side of subtraction
 \param Q2 Right side of subtraction
 \param QS Result of Q1 - Q2
 */
void quatsubtraction(const quaternion* Q1, const quaternion* Q2, quaternion* QS)
{
  QS->w = Q1->w - Q2->w;
  QS->x = Q1->x - Q2->x;
  QS->y = Q1->y - Q2->y;
  QS->z = Q1->z - Q2->z;

}
