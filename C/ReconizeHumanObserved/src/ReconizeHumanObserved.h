/*
 * ReconizeHumanObserved.h
 *
 *  Created on: Mar 30, 2015
 *      Author: BalaGangadhar
 */
#ifdef __cplusplus
extern "C" {
#endif
#ifndef RECONIZEHUMANOBSERVED_H_
#define RECONIZEHUMANOBSERVED_H_

#include "Human.h"
#include "XMLHumanPoseRead.h"
#include "XMLHumanActivityRead.h"
#include "MatchPatternToObserved.h"
#include "CacheUnit.h"
#include "ActivityPattern.h"
#include "ObservedSequence.h"
#include "Quaternion_Parsing.h"
#include "quaternion.h"

int classifier(char* activityFolder, quaternion quaternionSequence[][9], int windowLength, char* outputFile, int verboseLevel);

#endif /* RECONIZEHUMANOBSERVED_H_ */
#ifdef __cplusplus
}
#endif
