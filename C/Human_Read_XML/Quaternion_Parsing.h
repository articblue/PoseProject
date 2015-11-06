/*
 * Quaternion_Parsing.h
 *
 *  Created on: Mar 20, 2015
 *      Author: BalaGangadhar
 */

#ifndef QUATERNION_PARSING_H_
#define QUATERNION_PARSING_H_

#include "XMLCommon.h"
#include "XMLHumanPoseRead.h"
#include "ObservedSequence.h"
#include "ActivityPattern.h"
#include "Constants.h"
#include "quaternion.h"

READ_ERRORS quaternionParsing(quaternion quaternionSequence[][9] , int windowLength, ObservedSequence* O) ;

#endif /* QUATERNION_PARSING_H_ */
