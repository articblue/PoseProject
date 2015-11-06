/*
 * ActivityPattern.h
 *
 *  Created on: Aug 8, 2012
 *      Author: madisob
 */

#ifndef ACTIVITYPATTERN_H_
#define ACTIVITYPATTERN_H_

#include "Constants.h"
#include "Pattern.h"

//!  Structure for an Activities Pattern
/*!
 * Consist of a Pattern with a size P.N array of weights which signify the importance of each dimension of the character in the pattern
 *
 * Also consists of a unique identifier to identify the activity from others.
 * \sa Pattern
 */
typedef struct _ActivityPattern
{

	Pattern		P;						/*!< Pattern of Activity */

	real_num*		W;						/*!< Weights for each dimension of the characters. */
	int			id;						/*!< A unique integer to distuingish this activity from others. */

} ActivityPattern;

#endif /* ACTIVITYPATTERN_H_ */
