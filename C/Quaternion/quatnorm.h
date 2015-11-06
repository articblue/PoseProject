/*
 * quatnorm.h
 *
 *  Created on: Jul 11, 2012
 *      Author: madisob
 */
#ifndef QUATNORM_H_
#define QUATNORM_H_

#include "quaternion.h"

//!  Take the norm of a quaternion
/*
 \param Q  quaternion to take norm of
 \param QN result of || Q ||
 */
void quatnorm(const quaternion* , Q_real_num* );

#endif /* QUATNORM_H_ */
