/*
 * Sequence.h
 *
 *  Created on: Jan 23, 2013
 *      Author: madisob
 */

#ifndef SEQUENCE_H_
#define SEQUENCE_H_

#include "Constants.h"

//!  General Structure for a sequence of characters
/*!
 * A sequence is a size N array of characters.
 * Each character is of type CHARACTER.
 *
 * It is assumed that each character has been pulled from some stream.
 * sample_rate defines that streams sample rate, how much time exist between C[i] and C[i+1].
 */
typedef struct _Sequence
{
	real_num   		sample_rate;			/*!< Sample rate of the activity. */
	int     	N;    					/*!< Number of characters in pattern*/
	CHARACTER*	C;						/*!< Array of characters.*/

} Sequence;

//! Gets a sub sequence
/*!
 \param S Sequence to get sub-sequence of
 \param start the first character in sequence
 \param end the last character in sequence
 \return Sub-sequence of S
 */
Sequence subSequence(const Sequence* S, int start, int end);

#endif /* SEQUENCE_H_ */
