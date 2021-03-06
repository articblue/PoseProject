/*
 ============================================================================
 Name        : QuaternionTest.c
 Author      : Madison Blake
 Version     :
 Copyright   : Your copyright notice
 Description : Quaternion Test - Unit Test of Quaternion Library. Values arrived at using matlab
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include "quaternion.h"
#include "quatconj.h"
#include "quatproduct.h"
#include "quatsubtraction.h"
#include "quatadd.h"
#include "quatnorm.h"
#include "quatnormsquared.h"

int main(void)
{

  quaternion Q1, Q2;

  Q1.w = 1;
  Q1.x = 4;
  Q1.y = 7;
  Q1.z = 3;

  Q2.w = 6;
  Q2.x = 2;
  Q2.y = 4;
  Q2.z = 9;

  //Testing conjugate
  quaternion Qconj;
  quatconj(&Q1, &Qconj);
  if(Qconj.w != Q1.w || Qconj.x != -Q1.x || Qconj.y != -Q1.y || Qconj.z != -Q1.z)
    printf("ERROR: Conjugate Failure\n");

  //Testing Product
  quaternion Q12Prod;
  quatproduct(&Q1, &Q2, &Q12Prod);
  if(Q12Prod.w != -57 || Q12Prod.x != 77 || Q12Prod.y != 16 || Q12Prod.z != 29)
    printf("ERROR: Quaternion Product Failure\n");
  quaternion Q21Prod;
  quatproduct(&Q2, &Q1, &Q21Prod);
  if(Q21Prod.w != -57 || Q21Prod.x != -25 || Q21Prod.y != 76 || Q21Prod.z != 25)
    printf("ERROR: Quaternion Product Failure\n");

  //Testing Subtraction
  quaternion Q12Subtract;
  quatsubtraction(&Q1, &Q2, &Q12Subtract);
  if(Q12Subtract.w != -5 || Q12Subtract.x != 2 || Q12Subtract.y != 3 || Q12Subtract.z != -6)
    printf("ERROR: Quaternion Subtraction Failure\n");

  //Testing Addition
  quaternion Q12Add;
  quatAdd(&Q1, &Q2, &Q12Add);
  if(Q12Add.w != 7 || Q12Add.x != 6 || Q12Add.y != 11 || Q12Add.z != 12)
    printf("ERROR: Quaternion Addition Failure\n");

  //Testing norm
  Q_real_num Q1Norm;
  quatnorm(&Q1, &Q1Norm);
  if(Q1Norm > 8.66026 || Q1Norm < 8.66025)
    printf("ERROR: Quaternion Norm Failure\n");

  Q_real_num Q1SquaredNorm;
  quatnormsquared(&Q1, &Q1SquaredNorm);
  if(Q1SquaredNorm > pow(8.66026,2) || Q1SquaredNorm < pow(8.66025,2))
    printf("ERROR: Quaternion Norm Failure\n");

  //Testing Norm of Q1-Q2
  quaternion Q1MinusQ2;
  quatsubtraction(&Q1, &Q2, &Q1MinusQ2);
  Q_real_num Difference;
  quatnorm(&Q1MinusQ2, &Difference);
  if(Difference > 8.6024 || Difference < 8.6023)
    printf("ERROR: Quaternion Subtraction Norm Failure\n");

  printf("Done\n");
  return EXIT_SUCCESS;
}
