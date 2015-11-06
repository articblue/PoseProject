/*
 * ObservedSequence.h
 *
 *  Created on: Aug 15, 2012
 *      Author: madisob
 */

#ifndef OBSERVEDSEQUENCE_H_
#define OBSERVEDSEQUENCE_H_

#include "Constants.h"
#include "Sequence.h"

//!  Sequence of observed Characters.
/*!  An observed sequence is a sequence of characters followed by parameters to uniquely identify that sequence
 * \sa Sequence
 */
typedef struct _ObservedSequence
{
	Sequence	S;							/*!< Sequence that the observation is. */

	int			activity;					/*!< integer of the activity the sequence represents. */
	int			data_source;				/*!< integer of the source of the data for this sequence. */
	int			subject;					/*!< integer of the subject the data represents. */
	int			trial;						/*!< integer of the trial of the subject the data represents. */

} ObservedSequence;

#endif /* OBSERVEDSEQUENCE_H_ */
