/*
 * quatdot.h
 *
 *  Created on: Sep 11, 2012
 *      Author: madisob
 */

#ifndef QUATDOT_H_
#define QUATDOT_H_

#include "quaternion.h"

//!  Take the dot product of a quaterion
/*
 \param Q1 Left side of dot product
 \param Q2 Right side of dot product
 \param Qdot result of dot(Q1, Q2)
 */
void quatdot(const quaternion* Q1, const quaternion* Q2, Q_real_num* Qdot);

#endif /* QUATDOT_H_ */
