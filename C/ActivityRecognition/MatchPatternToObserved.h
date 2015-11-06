/*
 * RecognizeObserved.h
 *
 *  Created on: Aug 8, 2012
 *      Author: madisob
 */

#ifndef RECOGNIZEOBSERVED_H_
#define RECOGNIZEOBSERVED_H_

#include "CacheUnit.h"
#include "Pattern.h"
#include "Sequence.h"


//! Finds the score of some observation to an activity
/*!
 * Can save results in a chache so that results computed can be saved and used in a future calling
 *
 \param P 			Pattern to match against
 \param O 			Observed Character sequence
 \param cache 		triple pointer to save results to save time in the future, set to NULL if not using.
 \param *E 			Function pointer for computing the Error
 \param C			Two dim array for a section of memory to hold the cost for the string matching.
 \param D			Two dim array for a section of memory to hold the number of deletions done for string.
 \param maxScore	A score which to quit on, set to -1 to not use
 \return 			Score of how well the Observed sequence matches to the pattern
 \sa PatternString CharacterSequence CacheUnit
 */
real_num matchPatternToObserved(const Pattern* P, const Sequence* O, CacheUnit*** cache, void (*E)(const CHARACTER, const CHARACTER, real_num*), void (*Add)(const real_num*, const real_num*, real_num*), void (*Divide)(const real_num*, const real_num, real_num*), real_num (*Real)(const real_num*), real_num*** C, int_num** D, real_num* maxScore, void (*GetCache)(const real_num*, real_num*), real_num* Tmp_Space_Sub, real_num* Tmp_Space_Del, real_num* Final_Answer);

#endif /* RECOGNIZEOBSERVED_H_ */
