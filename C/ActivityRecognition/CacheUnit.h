/*
 * CacheUnit.h
 *
 *  Created on: Aug 9, 2012
 *      Author: madisob
 */

#ifndef CACHEUNIT_H_
#define CACHEUNIT_H_

#include "Constants.h"

//! A unit of cache for the system
typedef struct _CacheUnit
{
  real_num* 	value;          /*!< Value in cache. */
  int   set;            	/*!< indicates if set or not. */

} CacheUnit;


#endif /* CACHEUNIT_H_ */
