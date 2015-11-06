/*
 * quatnormsquared.c
 *
 *  Created on: Aug 22, 2012
 *      Author: madisob
 */

#include "quatnormsquared.h"
#include "quatconj.h"
#include "quatproduct.h"

//!  Finds the square of the norm of a quaternion
/*
 Using this is computationally more efficient than performing the square root of quatnorm()

 \param Q     quaternion to take norm of
 \param Qnorm Result of || Q ||^2
 */
void quatnormsquared(const quaternion* Q, Q_real_num* qnorms)
{
	  quaternion QConj, QProduct;
	  quatconj(Q, &QConj);
	  quatproduct(&QConj, Q, &QProduct);
	  *qnorms = QProduct.w;
}
