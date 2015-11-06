/*
 * quatnorm.c
 *
 *  Created on: Jul 11, 2012
 *      Author: madisob
 *
 *  Normalizes a quaternion
 */
#include <math.h>
#include "quatnorm.h"
#include "quatconj.h"
#include "quatproduct.h"

//!  Take the norm of a quaternion
/*
 \param Q  quaternion to take norm of
 \param QN result of || Q ||
 */
void quatnorm(const quaternion* Q, Q_real_num* QN)
{
  quaternion QConj, QProduct;
  quatconj(Q, &QConj);
  quatproduct(&QConj, Q, &QProduct);
  *QN = sqrt(QProduct.w);
}
