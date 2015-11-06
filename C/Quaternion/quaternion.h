/*
 * quaternion.h
 *
 *  Created on: Jul 11, 2012
 *      Author: madisob
 *
 *  Basic data structure for a quaternion
 */
#ifndef QUATERNION_H_
#define QUATERNION_H_

//!  Typedef for the datatype used to store the number in a quaternion
typedef double Q_real_num;

//!  Structure to represent a quaternion
typedef struct _quaternion
{
	Q_real_num w;				/*!< w component*/
	Q_real_num x;				/*!< x component*/
	Q_real_num y;				/*!< y component*/
	Q_real_num z;				/*!< z component */
} quaternion;

#endif /* QUATERNION_H_*/
