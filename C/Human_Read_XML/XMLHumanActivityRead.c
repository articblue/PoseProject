/*
 * XMLHumanActivityRead.c
 *
 *  Created on: Aug 14, 2012
 *      Author: madisob
 */
#include "XMLHumanActivityRead.h"
#include "Constants.h"
#include "Human.h"
#include <stdio.h>
#include <mxml.h>

READ_ERRORS XMLHumanActivityRead(const char* file, ActivityPattern* A)
{
	int i;
	FILE *fp;
	mxml_node_t *tree;

	float sample_rate;
	int N;
	POSE* C;
	real_num* min;
	real_num* max;
	real_num* W;

	int id;


	//construct XML tree
	fp = fopen(file, "r");
	if(fp == NULL)
	{
		printf("No File\n");
		return EVERYTHINGBROKE;
	}
	tree = mxmlLoadFile(NULL, fp, MXML_NO_CALLBACK);
	fclose(fp);

	//anyalize tree
	const char* str_sample_rate = mxmlElementGetAttr(tree, "sample_rate");
	sample_rate = atof(str_sample_rate);
	N = atoi(mxmlElementGetAttr(tree, "characters"));
	id = atoi(mxmlElementGetAttr(tree, "activity"));
	C = malloc(sizeof(POSE) * N);
	for(i = 0 ; i < N ; i++)
	{
		C[i] = malloc(sizeof(SPACE) * POSE_DIMENSION);
	}
	min = malloc(sizeof(real_num) * N);
	max = malloc(sizeof(real_num) * N);
	W = malloc(sizeof(real_num) * POSE_DIMENSION);

	//Read in weights
	mxml_node_t *weight = mxmlFindElement(tree, tree, "weights", NULL, NULL, MXML_DESCEND);
	while(weight != NULL)
	{
		W[atoi(mxmlElementGetAttr(weight, "index")) - 1] = atof(mxmlElementGetAttr(weight, "value"));
		weight = mxmlFindElement(weight, tree, "weights", NULL, NULL, MXML_DESCEND);
	}

	//read in characters
	int count = 0;
	mxml_node_t *character = mxmlFindElement(tree, tree, "character", NULL, NULL, MXML_DESCEND);
	while(character != NULL)
	{
		min[count] = atof(mxmlElementGetAttr(character, "min"));
		max[count] = atof(mxmlElementGetAttr(character, "max"));

		mxml_node_t* dim = mxmlFindElement(character, tree, "dimension", NULL, NULL, MXML_DESCEND_FIRST);
		while(dim != NULL)
		{
			int dim_index = atoi(mxmlElementGetAttr(dim, "index"));

			float Values[4];

			mxml_node_t* QA  = mxmlFindElement(dim, tree, "quaternion", NULL, NULL, MXML_DESCEND_FIRST);
			int QA_index = atoi(mxmlElementGetAttr(QA, "index"));
			Values[QA_index-1] = atof(mxmlElementGetAttr(QA, "value"));

			mxml_node_t* QB  = mxmlFindElement(QA, tree, "quaternion", NULL, NULL, MXML_NO_DESCEND);
			int QB_index = atoi(mxmlElementGetAttr(QB, "index"));
			Values[QB_index-1] = atof(mxmlElementGetAttr(QB, "value"));

			mxml_node_t* QC  = mxmlFindElement(QB, tree, "quaternion", NULL, NULL, MXML_NO_DESCEND);
			int QC_index = atoi(mxmlElementGetAttr(QC, "index"));
			Values[QC_index-1] = atof(mxmlElementGetAttr(QC, "value"));

			mxml_node_t* QD  = mxmlFindElement(QC, tree, "quaternion", NULL, NULL, MXML_NO_DESCEND);
			int QD_index = atoi(mxmlElementGetAttr(QD, "index"));
			Values[QD_index-1] = atof(mxmlElementGetAttr(QD, "value"));

			C[count][dim_index - 1].w = Values[0];
			C[count][dim_index - 1].x = Values[1];
			C[count][dim_index - 1].y = Values[2];
			C[count][dim_index - 1].z = Values[3];

			dim = mxmlFindElement(dim, tree, "dimension", NULL, NULL, MXML_NO_DESCEND);
		}

		count++;
		character = mxmlFindElement(character, tree, "character", NULL, NULL, MXML_DESCEND);
	}

	if(count != N)
	{
		mxmlDelete(tree);
		for(i = 0 ; i < N ; i++)
		{
			free(C[i]);
		}
		free(C);
		free(min);
		free(max);
		return EVERYTHINGBROKE;
	}

	//Add to Activity
	A->P.C = (CHARACTER*)C;
	A->P.N = N;
	A->P.sample_rate = sample_rate;
	A->P.min = min;
	A->P.max = max;
	A->W = W;
	A->id = id;

	//delete tree
	mxmlDelete(tree);

	return EVERYTHINGISOKAY;
}
