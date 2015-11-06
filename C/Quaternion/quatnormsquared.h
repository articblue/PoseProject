/*
 * quatnormsquared.h
 *
 *  Created on: Aug 22, 2012
 *      Author: madisob
 */

#ifndef QUATNORMSQUARED_H_
#define QUATNORMSQUARED_H_

#include "quaternion.h"

//!  Take the norm of a quaternion and square it
/*
 * Use this function if you need the square of the norm as it removes the need to perform
 * a square root operation.
 \param Q     quaternion to take norm of
 \param Qnorm Result of || Q ||^2
 */
void quatnormsquared(const quaternion* Q, Q_real_num* qnorms);


#endif /* QUATNORMSQUARED_H_ */
