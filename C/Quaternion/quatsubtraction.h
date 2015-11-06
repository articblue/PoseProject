/*
 * quatsubtraction.h
 *
 *  Created on: Jul 11, 2012
 *      Author: madisob
 */

#ifndef QUATSUBTRACTION_H_
#define QUATSUBTRACTION_H_

#include "quaternion.h"

//!  Subtract two quaternions
/*
 \param Q1 Left side of subtraction
 \param Q2 Right side of subtraction
 \param QS Result of Q1 - Q2
 */
void quatsubtraction(const quaternion* Q1, const quaternion* Q2, quaternion* QS);

#endif /* QUATSUBTRACTION_H_ */
