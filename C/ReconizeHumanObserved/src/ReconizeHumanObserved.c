/*
 ============================================================================
 Name        : ReconizeHumanObserved.c
 Author      : Madison Blake
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */

#include "ReconizeHumanObserved.h"

#define MAX_NUMBER_OF_OBSERVATIONS_DONE 3000
#define MAX_NUMBER_OF_ACTIVITIES 100

#include <stdio.h>
#include <stdlib.h>
#include <dirent.h>
#include <string.h>
#include <time.h>

#include "Human.h"
#include "XMLHumanPoseRead.h"
#include "XMLHumanActivityRead.h"
#include "MatchPatternToObserved.h"
#include "CacheUnit.h"
#include "ActivityPattern.h"
#include "ObservedSequence.h"
#include "Quaternion_Parsing.h"
#include "quaternion.h"

/* this is the definition of the anonymous function
 * LAMBDA EXPRESSIONS ARE AWSOME
 */
#define lambda(l_ret_type, l_arguments, l_body)         \
  ({                                                    \
	l_ret_type l_anonymous_functions_name l_arguments   \
	  l_body                                            \
	&l_anonymous_functions_name;                        \
  })


int classifier(char* activityFolder, quaternion quaternionSequence[][9], int windowLength, char* outputFile, int verboseLevel)
{
	int MatchCount = 0;
	float AvgStringCount = 0;

	int i, j, k, A_index, O_index;
	int O_count = 0;
	int A_count = 0;
	int verbose = verboseLevel;
	clock_t start_t, end_t;

	char* activity_folder;
	// char* observed_folder;
	char* output_file;

	FILE* fp;

	activity_folder = "/home/root/Activities/";  //same as the activityFolder argument
	// observed_folder = observedFolder;
	// output_file = outputFile;
	verbose = verboseLevel;

	// either get the output file from the arguments or create 3 separate files for each window to avoid overwriting
	if (windowLength == 30) {
		output_file = "/home/root/output1.txt";	
	} else if (windowLength == 60) {
		output_file = "/home/root/output2.txt";	
	} else {
		output_file = "/home/root/output3.txt";
	}

	fp = fopen(output_file, "a");

// previous code to take the input args
/*	if(argc == 1)
	{
		activity_folder = "Activities/";
		observed_folder = "Observed/";
		output_file = "output.txt";
	}
	if(argc >= 3)
	{
		activity_folder = (char*)argv[1];
		observed_folder = (char*)argv[2];
		output_file = "output.txt";
	}
	if(argc >= 4)
	{
		output_file = (char*)argv[3];
		fp = fopen(output_file, "w");
	}
	else
	{
		//fp = stdout;
		fp = fopen(output_file, "w");
	}

	//Verbose level
	// 0x1 -> print match of observed to each activity
	// 0x2 -> print summary of matches to each activity (min, max, avg)
	// 0x4 -> print best match and next best match for each observed
	if(argc >= 5)
	{
		verbose = atoi(argv[4]);
		printf("Verbose: %d\n", verbose);
	}
	else
		verbose = 255;
*/

	//All Observed Character Sequence
	ObservedSequence O[MAX_NUMBER_OF_OBSERVATIONS_DONE];

	//All activities to reconize
	ActivityPattern A[MAX_NUMBER_OF_ACTIVITIES];

	//construct 2 dim array to hold errors seen by observed activity matching to
	// i.e. activity_errors[1][i] for all i such that activity_errors[1][i] > 0
	//      is the error of all observations which were of activity 1;
	real_num activity_errors[MAX_NUMBER_OF_ACTIVITIES][MAX_NUMBER_OF_OBSERVATIONS_DONE];

	for(i = 0 ; i < MAX_NUMBER_OF_ACTIVITIES ; i++)
	{
		for(j = 0 ; j < MAX_NUMBER_OF_OBSERVATIONS_DONE ; j++)
		{
			activity_errors[i][j] = -1;
		}
	}

	//construct 3D array to hold the full vector of errors seen by observed activity matching to
	//Apparently this excedes the stack size, so I must create dynamically
	real_num*** activity_errors_full;
	activity_errors_full = malloc(sizeof(real_num**) * MAX_NUMBER_OF_ACTIVITIES);
	for(i = 0 ; i < MAX_NUMBER_OF_ACTIVITIES ; i++)
	{
		activity_errors_full[i] = malloc(sizeof(real_num*) * MAX_NUMBER_OF_OBSERVATIONS_DONE);
		for(j = 0 ; j < MAX_NUMBER_OF_OBSERVATIONS_DONE ; j++)
		{
			activity_errors_full[i][j] = malloc(sizeof(real_num)*POSE_DIMENSION);
		}
	}

	//construct 3 dim array to hold start and end for best match to each activity
	int best_match_start_end[MAX_NUMBER_OF_ACTIVITIES][MAX_NUMBER_OF_OBSERVATIONS_DONE][2];
	for(i = 0 ; i < MAX_NUMBER_OF_ACTIVITIES ; i++)
	{
		for(j = 0 ; j < MAX_NUMBER_OF_OBSERVATIONS_DONE ; j++)
		{
			best_match_start_end[i][j][0] = -1;
			best_match_start_end[i][j][1] = -1;
		}
	}


// previous code to parse the observed folder
/*	DIR* dp;
	struct dirent *ep;

	//Read from Observed folder
	dp = opendir(observed_folder);
	if (dp != NULL)
	{
		int count = 0;
		ep = readdir(dp);
		while (ep != NULL)
		{
			//skip first two entries ("." and "..") fore my code cant deal with that
			if(count < 2)
				count++;
			else
			{
				ObservedSequence O_tmp;
				char FILE[200] = "";
				strcat(FILE, observed_folder);
				strcat(FILE, ep->d_name);
				//printf("Observed File: %s\n", FILE);
				if(XMLHumanPoseRead(FILE, &O_tmp) == EVERYTHINGISOKAY)
				{
					O[O_count] = O_tmp;
					O_count++;
				}
			}

			ep = readdir(dp);
		}
		(void) closedir (dp);
	}
	else
	{
		printf ("Couldn't open Observed directory");
		return EXIT_FAILURE;
	}*/


	ObservedSequence O_tmp ;
	if((quaternionParsing(quaternionSequence, windowLength, &O_tmp) == EVERYTHINGISOKAY))
	{
		O[O_count] = O_tmp;
		O_count++;
	} else {
		printf("unable to call the quaternionParsing function");
	}

	//Read from Activity folder

	DIR* dp;
	struct dirent *ep;
	dp = opendir(activity_folder);
	if (dp != NULL)
	{
		ep = readdir(dp);
		while (ep != NULL)
		{
			if(strcmp(ep->d_name , "..") == 0)
			{
				ep = readdir(dp);
				continue;
			}

			if(strcmp(ep->d_name, ".") == 0)
			{
				ep = readdir(dp);
				continue;
			}


			ActivityPattern A_tmp;
			char FILE[200] = "";
			strcat(FILE, activity_folder);
			strcat(FILE, ep->d_name);
			// printf("Activity File: %s\n", FILE);
			// fflush(stdout);
			if(XMLHumanActivityRead(FILE, &A_tmp) == EVERYTHINGISOKAY)
			{
				// printf("  Sample Rate: %f\n", A_tmp.P.sample_rate);
				// fflush(stdout);
				A[A_count] = A_tmp;
				A_count++;
			}

			ep = readdir(dp);

		}
		(void) closedir (dp);
	}
	else
	{
		printf ("Couldn't open Activities directory");
		return EXIT_FAILURE;
	}


	// printf("%d Observations\n", O_count);
	// printf("%d Activities\n", A_count);
	// fflush(stdout);

	start_t = clock();

	for(O_index = 0 ; O_index < O_count; O_index++)
	{
		if((verbose & 0x1) == 1)
			fprintf(fp, "%d Observed Activity:%d DataSource:%d Subject:%d Trial:%d\n", O_index, O[O_index].activity, O[O_index].data_source, O[O_index].subject, O[O_index].trial);

		for(A_index = 0 ; A_index < A_count ; A_index++)
		{
			//compute max length
			int max_activity_length = 0;
			for(i = 0 ; i < A[A_index].P.N ; i++)
			{
				max_activity_length += A[A_index].P.max[i];
			}

			//compute min length
			int min_activity_length = 0;
			for(i = 0 ; i < A[A_index].P.N ; i++)
			{
				min_activity_length += A[A_index].P.min[i];
			}

			//Construct Cache
			//Cache is size of pattern by maximum possible string

			CacheUnit ***cache = malloc( max_activity_length * sizeof(CacheUnit**));
			for(i = 0 ; i < max_activity_length ; i++)
			{
				cache[i] = malloc(A[A_index].P.N * sizeof(CacheUnit*));
				for(j = 0 ; j < A[A_index].P.N ; j++)
				{
					cache[i][j] = malloc(sizeof(CacheUnit));
					cache[i][j]->value = malloc(sizeof(real_num)*POSE_DIMENSION);
					cache[i][j]->set = 0;
					for(k = 0 ; k < POSE_DIMENSION ; k++)
					{
						cache[i][j]->value[k] = -1.0;
					}
				}
			}

			int check;
			int number_of_checks = O[O_index].S.N + max_activity_length;
			float* errors = malloc(sizeof(float) * number_of_checks);
			float** errors_fullArr = malloc(sizeof(float*) * number_of_checks);
			int* start_arr  = malloc(sizeof(int)   * number_of_checks);
			int* end_arr    = malloc(sizeof(int)   * number_of_checks);


			for(check = 0 ; check < number_of_checks ; check++)
			{
				//find where the start of the substring needs to be
				int start;
				if(check < max_activity_length)
					start = 0;
				else
					start = check - max_activity_length;

				int end;
				if(check >= O[O_index].S.N)
					end = O[O_index].S.N;
				else
					end = check;

				Sequence sub_O = subSequence(&O[O_index].S, start, end);

				//because we are not starting 0 it means we incrimented our starting point
				//so we must move the cache
				if(start > 0)
				{

					//save off pointer for unessessary column
					CacheUnit** Tmp = cache[0];

					//shift cache all to the left
					for (j = 0  ; j < max_activity_length - 1 ; j++)
					{
						cache[j] = cache[j + 1];
					}

					//Set last column to the temporay and assign all as unset
					cache[max_activity_length - 1] = Tmp;
					for(j = 0 ; j < A[A_index].P.N ; j++)
					{
						cache[max_activity_length - 1][j]->set = 0;
					}

				}

				//Dynamically allocate C and D arrays
				real_num*** C = malloc(sizeof(real_num**) * A[A_index].P.N);
				int** D = malloc(sizeof(int*) * A[A_index].P.N);
				for(i = 0 ; i < A[A_index].P.N ; i++)
				{
					C[i] = malloc(sizeof(real_num*) * sub_O.N);
					D[i] = malloc(sizeof(int) * sub_O.N);
					for(j = 0 ; j < sub_O.N ; j++)
					{
						D[i][j] = 0;
						C[i][j] = malloc(sizeof(real_num) * POSE_DIMENSION);
						for(k = 0 ; k < POSE_DIMENSION ; k++)
						{
							C[i][j][k] = 0.0;
						}
					}
				}


				//set up lamda function for error function
				void (*Err)(const CHARACTER, const CHARACTER, real_num*) = lambda(void, (const CHARACTER C1, const CHARACTER C2, real_num* V), { humanError((POSE)C1, (POSE)C2, A[A_index].W, V); });

				//set up function to add and find real value on the error of a human
				void (*Add)(const real_num*, const real_num*, real_num*) = humanAddError;
				real_num (*Real)(const real_num* E) = humanRealError;
				void (*Divide)(const real_num* quotient, const real_num divisor, real_num* answer) = humanDivideError;

				void (*SetCache)(const real_num*, real_num*) = lambda(void, (const real_num* From, real_num* To), 
					{
						int i;
						for (i = 0 ; i < POSE_DIMENSION ; i++)
						{
							To[i] = From[i];
						}
					});

				real_num* TmpSpace1 = malloc(sizeof(real_num) * POSE_DIMENSION);
				real_num* TmpSpace2 = malloc(sizeof(real_num) * POSE_DIMENSION);

				//allocate space for full array
				errors_fullArr[check] = malloc(sizeof(real_num) * POSE_DIMENSION);

				//Compute Error
				//errors[check] = matchPatternToObserved(&A[A_index].P, &sub_O, cache, Err, C, D, 0);

				errors[check] = matchPatternToObserved(&A[A_index].P, &sub_O, cache, Err, Add, Divide, Real, C, D, 0, SetCache,TmpSpace1, TmpSpace2, errors_fullArr[check]);

				MatchCount++;
				AvgStringCount = (((float)AvgStringCount)*((float)MatchCount-1)/((float)MatchCount) + ((float)sub_O.N)/((float)MatchCount));



				start_arr[check] = start;
				end_arr[check] = end;

				//free up allocated stuff
				free(TmpSpace1);
				free(TmpSpace2);

				for(i = 0 ; i < A[A_index].P.N ; i++)
				{
					for(j = 0 ; j < sub_O.N ; j++)
					{
						free(C[i][j]);
					}
					free(C[i]);
					free(D[i]);
				}
				free(C);
				free(D);
			}

			int min_error_index = -1;
			int an_error_found = 0;
			for(i = 0 ; i < number_of_checks ; i++)
			{
				if(an_error_found == 0 && errors[i] > 0.0)
				{
					min_error_index = i;
					an_error_found = 1;
				}
				else if(errors[i] > 0.0 && errors[i] < errors[min_error_index])
					min_error_index = i;
			}


			if(an_error_found == 0)
			{
				activity_errors[A_index][O_index] = -1;
				best_match_start_end[A_index][O_index][0] = -1;
				best_match_start_end[A_index][O_index][1] = -1;
			}
			else
			{
				//divide by 1 (really just a copy)
				
				humanDivideError(errors_fullArr[min_error_index], 1.0, activity_errors_full[A_index][O_index]);

				activity_errors[A_index][O_index] = errors[min_error_index];
				best_match_start_end[A_index][O_index][0] = start_arr[min_error_index];
				best_match_start_end[A_index][O_index][1] = end_arr[min_error_index];
			}

			if((verbose & 0x1) == 1)
				fprintf(fp, "  Activity %d: %10.2f | %d %d\n", A[A_index].id, activity_errors[A_index][O_index], best_match_start_end[A_index][O_index][0], best_match_start_end[A_index][O_index][1]);

			//free up stuff

			for(check = 0 ; check < number_of_checks ; check++)
			{
				free(errors_fullArr[check]);
			}

			for(i = 0 ; i < max_activity_length ; i++)
			{
				for(j = 0 ; j < A[A_index].P.N ; j++)
				{

					free(cache[i][j]->value);
					free(cache[i][j]);
				}
				free(cache[i]);
			}
			free(cache);
			free(errors);
			free(errors_fullArr);
		}
	}

	//free up stuff in observed array
	for(i = 0 ; i < O_count; i++)
	{	
		for(j = 0 ; j < O[i].S.N ; j++)
		{
			free((void*)O[i].S.C[j]);
		}
		free(O[i].S.C);
	}


	//free up stuff in activity array
	for(i = 0 ; i < A_count; i++)
	{
		for(j = 0 ; j < A[i].P.N ; j++)
		{
			free((void*)A[i].P.C[j]);
		}
		free(A[i].P.C);
		free(A[i].W);
		free(A[i].P.max);
		free(A[i].P.min);
	}

	end_t = clock();
	double executionTime = (double)(end_t-start_t)/CLOCKS_PER_SEC;

	// printf("Execution time of %f sec \n", executionTime);
	// fflush(stdout);

	//print off matches and not matches
	if((verbose & 0x2) >> 1 == 1)
	{
		for(A_index = 0 ; A_index < A_count ; A_index++)
		{
			printf("Examining match to activity %d\n", A[A_index].id);

			real_num min_match = 1000000000000;
			real_num sum_match = 0;
			real_num max_match = 0;
			int count_match = 0;

			real_num min_not_match = 1000000000000;
			real_num sum_not_match = 0;
			real_num max_not_match = 0;
			int count_not_match = 0;

			for(O_index = 0 ; O_index < MAX_NUMBER_OF_OBSERVATIONS_DONE ; O_index++)
			{
				if(activity_errors[A_index][O_index] < 0)
					continue;

				if(A[A_index].id == O[O_index].activity)
				{
					if(min_match > activity_errors[A_index][O_index])
						min_match = activity_errors[A_index][O_index];
					if(max_match < activity_errors[A_index][O_index])
						max_match = activity_errors[A_index][O_index];
					sum_match += activity_errors[A_index][O_index];
					count_match++;
				}
				else
				{
					if(min_not_match > activity_errors[A_index][O_index])
						min_not_match = activity_errors[A_index][O_index];
					if(max_not_match < activity_errors[A_index][O_index])
						max_not_match = activity_errors[A_index][O_index];
					sum_not_match += activity_errors[A_index][O_index];
					count_not_match++;
				}

			}

			printf("  Of %d matches \n    avg: %f \n    min: %f \n    max: %f\n", count_match, sum_match/count_match, min_match, max_match);
			printf("  Of %d not matches \n    avg: %f \n    min: %f \n    max: %f\n", count_not_match, sum_not_match/count_not_match, min_not_match, max_not_match);


			fprintf(fp, "%f %f %f\n", sum_match/count_match, min_match, max_match);
			fprintf(fp, "%f %f %f\n", sum_not_match/count_not_match, min_not_match, max_not_match);

		}
	}

	//Min Match and next best print off
	if((verbose & 0x4) >> 2 == 1)
	{
		for(O_index = 0 ; O_index < O_count ; O_index++)
		{
			int   bestMatchIndex =  -1;
			float bestMatchValue =  0;
			int   bestMatchIndex2 = -1;
			float bestMatchValue2 = 0;

			//Loop Through Activities
			for(A_index = 0 ; A_index < A_count; A_index++)
			{
				//If no match found then skip
				if(activity_errors[A_index][O_index] < 0)
					continue;

				//Best Match isn't set (when A_index == 0)
				if(bestMatchIndex == -1)
				{
					bestMatchIndex = A_index;
					bestMatchValue = activity_errors[A_index][O_index];
					continue;
				}

				//Second best match isn't set (when A_index == 1)
				else if(bestMatchIndex2 == -1)
				{
					if(bestMatchValue > activity_errors[A_index][O_index])
					{
						bestMatchIndex2 = bestMatchIndex;
						bestMatchValue2 = bestMatchValue;
						bestMatchIndex = A_index;
						bestMatchValue = activity_errors[A_index][O_index];
					}
					else
					{
						bestMatchIndex2 = A_index;
						bestMatchValue2 = activity_errors[A_index][O_index];
					}
					continue;
				}

				//If the A_index is a better match
				else if(bestMatchValue > activity_errors[A_index][O_index])
				{
					bestMatchIndex2 = bestMatchIndex;
					bestMatchValue2 = bestMatchValue;
					bestMatchIndex = A_index;
					bestMatchValue = activity_errors[A_index][O_index];
				}

				//If the A_index match is a second best match
				else if(bestMatchValue2 > activity_errors[A_index][O_index])
				{
					bestMatchIndex2 = A_index;
					bestMatchValue2 = activity_errors[A_index][O_index];
				}
			}

			// if(bestMatchIndex == -1)
				fprintf(fp, "%3d %3d %3d %3d | --         -- | --         -- |\n", O[O_index].activity, O[O_index].data_source, O[O_index].subject, O[O_index].trial);
			// else if(bestMatchIndex2 == -1)
				fprintf(fp, "%3d %3d %3d %3d | %2d %10.2f | --         -- |\n", O[O_index].activity, O[O_index].data_source, O[O_index].subject, O[O_index].trial, A[bestMatchIndex].id, bestMatchValue);
			// else
				fprintf(fp, "%3d %3d %3d %3d | %2d %10.2f | %2d %10.2f |\n", O[O_index].activity, O[O_index].data_source, O[O_index].subject, O[O_index].trial, A[bestMatchIndex].id, bestMatchValue, A[bestMatchIndex2].id, bestMatchValue2);
			//printf("%3d %3d %3d %3d & %10.2f & %2d & %10.2f\n", O[O_index].activity, O[O_index].data_source, O[O_index].subject, O[O_index].trial, bestMatchValue, A[bestMatchIndex2].id, bestMatchValue2);

		}
	}

	int final_activity = 0; // classified activity
	real_num min_score = 0; // score of the activity classified
	//Print off each match in an easy to parse format
	if((verbose & 0x8) >> 3 == 1)
	{	
		for(O_index = 0 ; O_index < O_count ; O_index++)
		{
			//Loop Through Activities
			min_score = 1000; //initially set a high value to compare the scores
			for(A_index = 0 ; A_index < A_count; A_index++)
			{
				fprintf(fp, "%d %d %d %d | %d | %f |", O[O_index].activity, O[O_index].data_source, O[O_index].subject, O[O_index].trial, A[A_index].id, activity_errors[A_index][O_index]);

				if(activity_errors[A_index][O_index] != -1.000000) {
					if( activity_errors[A_index][O_index] < min_score) {
						min_score = activity_errors[A_index][O_index];
						final_activity = A[A_index].id;
					}
				}

				/* for(i = 0 ; i < POSE_DIMENSION ; i++)
				{
					if(i != 0)
						fprintf(fp, " ");
					fprintf(fp, "%f", activity_errors_full[A_index][O_index][i]);
					
				} */
				
				fprintf(fp, "\n");
			}
		}
	}

	// free the activity errors 
	for(i = 0 ; i < MAX_NUMBER_OF_ACTIVITIES ; i++)
	{
		for(j = 0 ; j < MAX_NUMBER_OF_OBSERVATIONS_DONE ; j++)
		{
			free(activity_errors_full[i][j]);
		}
		free(activity_errors_full[i]);
	}

	free(activity_errors_full);

	// printf("Num Edits: %d\n", (int)MatchCount);
	// printf("Avg Char Size: %f\n", AvgStringCount);
	// printf("final activity: %d and the min_score: %f \n", final_activity, min_score);
	// fflush(stdout);

	if(fp != stdout)
	{
		// printf("Closing File \n");
		fclose(fp);
	}
	// printf("DONE!\n");
	return final_activity;
}
