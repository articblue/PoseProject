/*
 * quatadd.h
 *
 *  Created on: Sep 11, 2012
 *      Author: madisob
 */

#ifndef QUATADD_H_
#define QUATADD_H_

#include "quaternion.h"

//!  Adds two quaternions
/*
 \param Q1 Left side of addition
 \param Q2 Right side of addition
 \param Qresult Result of Q1 + Q2
 */
void quatAdd(const quaternion* Q1, const quaternion* Q2, quaternion* Qresult);

#endif /* QUATADD_H_ */
