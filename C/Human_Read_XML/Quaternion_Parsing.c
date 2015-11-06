/*
 * Quaternion_Parsing.c
 *
 *  Created on: Mar 10, 2015
 *      Author: BalaGangadhar
 */

#include "Quaternion_Parsing.h"
#include "XMLCommon.h"
#include "ObservedSequence.h"
#include "Constants.h"
#include "quaternion.h"

#include <stdio.h>
#include <stdlib.h>

//function to convert the quaternion Sequence into Observed Sequence structure

READ_ERRORS quaternionParsing(quaternion quaternionSequence[][9] , int windowLength, ObservedSequence* O) {

	POSE* C;

	int activity = 49; //random to fill the structure
	int data_source = 9; //random to fill the structure
	int subject = 99; //random to fill the structure
	int trial = 1; //random to fill the structure
	int sample_rate = 30;
	int N = (windowLength);
	int i = 0;
	int j = 0;
	int pose = 0;

	C = malloc(sizeof(POSE) * (N));
	for(j = 0 ; j < N ; j++)
	{
		C[j] = malloc(sizeof(SPACE) * 9);
	}


	for(pose = 0; pose < N; pose++) {
		for(i = 0; i < 9; i++) {
			C[pose][i].w = quaternionSequence[pose][i].w;
			C[pose][i].x = quaternionSequence[pose][i].x;
			C[pose][i].y = quaternionSequence[pose][i].y;
			C[pose][i].z = quaternionSequence[pose][i].z;
		}
	}


		// populate the structure	
	        O->S.C = (CHARACTER*)C;
		O->S.N = N;
		O->S.sample_rate = sample_rate;
		O->activity = activity;
		O->data_source = data_source;
		O->subject = subject;
		O->trial = trial;

		return EVERYTHINGISOKAY;
}

