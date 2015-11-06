/*
 * quatproduct.c
 *
 *  Created on: Jul 11, 2012
 *      Author: madisob
 *
 *  Finds the product of two quaternions
 */
#include "quatproduct.h"

//!  Take the product of two quaternions
/*
 \param Q1      Left side of product
 \param Q2      Right side of product
 \param product Result of Q1 * Q2
 */
void quatproduct(const quaternion* Q1, const quaternion* Q2, quaternion* product)
{

  product->w = Q1->w*Q2->w - Q1->x*Q2->x - Q1->y*Q2->y - Q1->z*Q2->z;
  product->x = Q1->w*Q2->x + Q1->x*Q2->w + Q1->y*Q2->z - Q1->z*Q2->y;
  product->y = Q1->w*Q2->y - Q1->x*Q2->z + Q1->y*Q2->w + Q1->z*Q2->x;
  product->z = Q1->w*Q2->z + Q1->x*Q2->y - Q1->y*Q2->x + Q1->z*Q2->w;


}

