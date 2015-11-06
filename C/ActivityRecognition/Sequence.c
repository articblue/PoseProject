/*
 * Sequence.c
 *
 *  Created on: Jan 31, 2013
 *      Author: madisob
 */

#include "Sequence.h"


//! Gets a sub sequence
/*!
 \param S Sequence to get sub-sequence of
 \param start the first character in sequence
 \param end the last character in sequence
 \return Sub-sequence of S
 */
Sequence subSequence(const Sequence* S, int start, int end)
{
	Sequence new_sequence;
	new_sequence.sample_rate = S->sample_rate;
	new_sequence.N = end - start;
	new_sequence.C = (S->C + start);

	return new_sequence;
}
