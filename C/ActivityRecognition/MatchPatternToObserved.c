/*
 * MatchObservedToActivityAlphabet.c
 *
 *  Created on: Aug 9, 2012
 *      Author: madisob
 */



#include "MatchPatternToObserved.h"

#include "Constants.h"
#include <stdio.h>

#if DEBUG_OUTPUT > 0
	#include <stdio.h>
#endif

#ifndef max
	#define max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef min
	#define min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif


int_num sumFloorEach(real_num* A, int_num S, int_num E, real_num div);

//! Finds the score of some observation to an activity
/*!
 * Finds the edit distance of some Pattern sequence P to some Observed sequence O.
 *
 * To cost of an edit is determined by E, a function pointer passed to this function.
 * The edit distance is the cost of editing sequence O to the regular expression expressed by P.
 * If the cost goes above a certian amount (maxScore) then the editing stops, this allows a means to "give up", saving execution time.'
 *
 * If edit cost can be pre-computed and passed in the cache matrix.
 * if cache[i][j]->set is true then E(P->C[i], O->C[j]) = cache[i][j]->value
 * That is, if the cache at [i,j] is set then the cost of editing the observed character j to the Pattern character i is cache[i][j]->value
 *
 * The purpose of the chache is to elimenate unessessary computation.
 * Since this algorithm is targeted to a real time system the observed sequence at time t is the same as t-1 only with one new character (frame) added on.
 * Therefore the chache from t-1 can be shifted, and its values can be used to reduce computation time (at the cost of memory)
 *
 \param P 			Pattern to match against
 \param O 			Observed Character sequence
 \param cache 		A space to save results and re-use already computed results to save time in the future, set to NULL if not using.
 \param *E 			Function pointer for computing the Error
 \param C			Two dim array for a section of memory to hold the cost for the string matching. Needs to be P->N X O->N.
 \param D			Two dim array for a section of memory to hold the number of deletions done for string. Needs to be P->N X O->N.
 \param maxScore	A score which to quit on, set to NULL to not use
 \return 			Score of how well the Observed sequence matches to the pattern
 \sa Pattern Sequence CacheUnit CHARACTER
 */
real_num matchPatternToObserved(const Pattern* P, const Sequence* O, CacheUnit*** cache, void (*E)(const CHARACTER, const CHARACTER, real_num*), void (*Add)(const real_num*, const real_num*, real_num*), void (*Divide)(const real_num*, const real_num, real_num*), real_num (*Real)(const real_num*), real_num*** C, int_num** D, real_num* maxScore, void (*GetCache)(const real_num*, real_num*), real_num* Tmp_Space_Sub, real_num* Tmp_Space_Del, real_num* final_Number)
{

	int_num i;
	int_num j;

	real_num sample_rate_ratio = P->sample_rate / O->sample_rate;

	//Initialize C and D
	//This may be unessessary
	for(i = 0 ; i < P->N ; i++)
	{
		for(j = 0 ; j < O->N ; j++)
		{
			//C[i][j] = 0;
			D[i][j] = 0;
		}
	}

	//Loop each character in Pattern
	int_num startAt = 0;
	for(i = 0 ; i < P->N ; i++)
	{
#if DEBUG_OUTPUT == 2
		for(j = 0 ; j < startAt ; j++)
		{
			printf("              -|");
		}
#endif

		//Loop across each character in observation.
		//Since no insertions are allowed we can skip ahead as we move along the pattern's characters (handled by startAt variable)
		int_num value_found = 0;
		for(j = startAt; j < O->N ; j++)
		{
			int_num deletion_set = 0;
			int_num substitution_set = 0;
			real_num deletion_cost = -1;
			real_num substitution_cost = -1;

			// initial case
			if(i == 0 && j == 0)
			{
				E(P->C[0], O->C[0], C[0][0]);
				D[0][0] = 1;
				value_found = 1;

#if DEBUG_OUTPUT == 1
				printf("Start - C[0][0] = %f\n", Real(C[0][0]));
#endif
#if DEBUG_OUTPUT == 2
				printf("%15.1f|", Real(C[0][0]));
#endif

				continue;
			}

			// ############## Check if deletion or substitution legal and record its cost ######################
			// deletion legal
			// first check that not along left side
			// Then check that a sub has been done on the left
			// Then check that the maximum number of insertions has not been performed (take the ceil)
			if(j > 0 && D[i][j-1] > 0 && D[i][j-1] < (int_num)(P->max[i] / sample_rate_ratio + 1))
			{
				//check cache
				if(cache == 0)
					E(P->C[i], O->C[j], Tmp_Space_Del);
				else
				{
					if(cache[j][i]->set == 0)
					{
						E(P->C[i], O->C[j], Tmp_Space_Del);
						GetCache(Tmp_Space_Del, cache[j][i]->value);
						cache[j][i]->set = 1;
					}
					else
					{
						GetCache(cache[j][i]->value, Tmp_Space_Del);
					}
				}
				
				Add(C[i][j-1], Tmp_Space_Del, C[i][j]);
				deletion_cost = Real(C[i][j]);
				deletion_set = 1;

				//if the new cost excedes the maximum then we cant do it
				if(maxScore != 0 && *maxScore < deletion_cost)
				{
					deletion_set = 0;
				}
			}

			//Substitution checking
			//First to make sure not on top or left edge
			//Then check if the spot comming from has done the minimum number of deletions
			//  We must round down P->min[i-1] / sample_rate_ratio
			//
			//C[i][j] is used as memory to do calculations on, the value placed in C[i][j] is in no way going to be the
			//final value placed here.
			if(i > 0 && j > 0 && D[i-1][j-1] >= max(1, (int_num)(P->min[i-1] / sample_rate_ratio)))
			{

				//if doing a substitution means we will eventually run out of room,
				//don't do it
				if(O->N - j >= sumFloorEach(P->min, i, P->N, sample_rate_ratio))
				{
					//Check Cache
					if(cache == 0)
						E(P->C[i], O->C[j], Tmp_Space_Sub);
					else
					{
						if(cache[j][i]->set == 0)
						{
							E(P->C[i], O->C[j], Tmp_Space_Sub);
							GetCache(Tmp_Space_Sub, cache[j][i]->value);
							cache[j][i]->set = 1;
						}
						else
						{
							GetCache(cache[j][i]->value, Tmp_Space_Sub);
						}
					}
					
					Add(C[i-1][j-1], Tmp_Space_Sub, C[i][j]);
					substitution_cost = Real(C[i][j]);
					substitution_set = 1;

					//if the new cost excedes the maximum then we cant do it
					if(maxScore != 0 && *maxScore < deletion_cost)
					{
						substitution_set = 0;
					}
				}
			}

			// ################# Decide which to pick and apply ############################
			// Four possibilities
			// 	1) Neither deletion or substitution can be done
			//  2) deletion can be done but substitution cant
			//  3) substitution can be done but deletion cant
			//  4) both substitution and deletion can be done
			//
			//if both are still unset then we
			//    1: set the start at point and continue
			//    2: or if already found data skip the rest of the row
			if(deletion_set == 0 && substitution_set == 0)
			{
#if DEBUG_OUTPUT == 2
				printf("              -|");
#endif

				if(value_found == 0)
				{
					startAt = j + 1;
					continue;
				}
				else
					break;
			}

			//if either is unset then it must be the other one
			if(deletion_set == 0)
			{
				Add(C[i-1][j-1], Tmp_Space_Sub, C[i][j]);
				value_found = 1;
				D[i][j] = 1;
#if DEBUG_OUTPUT == 1
				printf("Sub - C[%d][%d] = %f D[%d][%d] = %d\n",i, j, Real(C[i][j]), i, j, D[i][j]);
#endif
#if DEBUG_OUTPUT == 2
				printf("%15.1f|", Real(C[i][j]));
#endif

				continue;
			}
			if(substitution_set == 0)
			{
				Add(C[i][j-1], Tmp_Space_Del, C[i][j]);
				value_found = 1;
				D[i][j] = D[i][j-1] + 1;
#if DEBUG_OUTPUT == 1
				printf("Ins - C[%d][%d] = %f D[%d][%d] = %d\n",i, j, Real(C[i][j]), i, j, D[i][j]);
#endif
#if DEBUG_OUTPUT == 2
				printf("%15.1f|", Real(C[i][j]));
#endif

				continue;
			}

			//both are set so pick the min
			if(substitution_cost < deletion_cost)
			{
				Add(C[i-1][j-1], Tmp_Space_Sub, C[i][j]);
				value_found = 1;
				D[i][j] = 1;
#if DEBUG_OUTPUT == 1
				printf("Sub - C[%d][%d] = %f D[%d][%d] = %d\n",i, j, Real(C[i][j]), i, j, D[i][j]);
#endif
#if DEBUG_OUTPUT == 2
				printf("%15.1f|", Real(C[i][j]));
#endif
				continue;
			}
			else
			{
				Add(C[i][j-1], Tmp_Space_Del, C[i][j]);
				D[i][j] = D[i][j-1] + 1;
#if DEBUG_OUTPUT == 1
				printf("Ins - C[%d][%d] = %f D[%d][%d] = %d\n",i, j, Real(C[i][j]), i, j, D[i][j]);
#endif
#if DEBUG_OUTPUT == 2
				printf("%15.1f|", Real(C[i][j]));
#endif
				value_found = 1;
				continue;
			}
		}
#if DEBUG_OUTPUT == 2
		printf("\n");
#endif
	}

	//now we must find the solution
	//its along the bottom but not nessessarly at the right edge due to the solution could be in a substring
	char solution_found = 0;
	real_num min_solution = 0;
	for(i = 0 ; i < O->N ; i++)
	{
		if(D[P->N - 1][i] >= (int_num)(((P->min[P->N - 1] / sample_rate_ratio) +0.0001)))
		{
			if(solution_found == 0  && Real(C[P->N - 1][i]) >= 0.0)
			{
				Divide(C[P->N - 1][i], ((i+1) * sample_rate_ratio), final_Number);
				min_solution = Real(C[P->N - 1][i])/((i+1) * sample_rate_ratio);
				//min_solution = Real(C[P->N - 1][i]);
				solution_found = 1;
			}
			else if(min_solution > Real(C[P->N - 1][i])/((i+1) * sample_rate_ratio) && Real(C[P->N - 1][i]) >= 0)
			//else if(min_solution > Real(C[P->N - 1][i]) && Real(C[P->N - 1][i]) >= 0)
			{
				Divide(C[P->N - 1][i], ((i+1) * sample_rate_ratio), final_Number);
				min_solution = Real(C[P->N - 1][i])/((i+1) * sample_rate_ratio);
				//min_solution = Real(C[P->N - 1][i]);
			}
		}
	}

	//Find Edit distance
	real_num ed;
	if(solution_found == 1)
		ed = min_solution;
	else
		ed = -1;

	return ed;
}

//! Floor's each element in a vector and adds up
/*!
 \param A 		Array to sum
 \param S 		Start
 \param E 		End
 \param div 	NUM to divide each element by
 \return 		Sum of the floor of each element
 */
int_num sumFloorEach(real_num* A, int_num S, int_num E, real_num div)
{
	int_num i;
	int_num V = 0;
	for(i = S ; i < E ; i++)
	{
		V += max(1, (int_num)(A[i] / div));
	}

	return V;
}
