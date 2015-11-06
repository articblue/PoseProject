/*
 * Pattern.h
 *
 *  Created on: Jan 31, 2013
 *      Author: madisob
 */

#ifndef PATTERN_H_
#define PATTERN_H_

//!  A regular expression to express a set of possible sequences.
/*!
 * A Pattern expresses a set of sequences which together represent the regular expression of the pattern
 * Using information in the structure a regular expression can be constructed to match against.
 *
 * A sequence is a size N array of characters.
 * Each character is of type CHARACTER.
 *
 * It is assumed that each character has been pulled from some stream.
 * sample_rate defines that streams sample rate, how much time exist between C[i] and C[i+1].
 *
 * Each character is allowed a minimum and maximum occurences in the regular expression.
 *
 * The regex for possible sequences (assuming matching sample_rate's) for the activity is expressed by:
 * (C[1]^min[1] + | C[1]^1 | C[1]^2 | ... | C[1]^(max[1] - min[1])) + (C[2]^min[2] + | C[2]^1 | C[2]^2 | ... | C[2]^(max[2] - min[2])) + ...
 */
typedef struct _Pattern
{
	real_num    sample_rate;			/*!< Sample rate of the activity. */
	int     	N;    					/*!< Number of characters in pattern*/
	CHARACTER*	C;						/*!< The string of characters.*/

	real_num*		min;					/*!< Size N array of the minimum times each character can be repeated. */
	real_num*		max;					/*!< Size N array of the maximum times each character can be repeated. */

} Pattern;

#endif /* PATTERN_H_ */
