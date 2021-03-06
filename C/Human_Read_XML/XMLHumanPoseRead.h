/*
 * XMLHumanPoseRead.h
 *
 *  Created on: Aug 10, 2012
 *      Author: madisob
 */

#ifndef XMLHUMANPOSEREAD_H_
#define XMLHUMANPOSEREAD_H_

#include "XMLCommon.h"
#include "Human.h"
#include "ObservedSequence.h"

//! Reads xml file and converts to a sequence of poses
/*!
  Reads an xml file which contains some animation of an activity being performed.
  The XML file is parsed and the sequence P is filled out
  \param file Filename of the XML file
  \param S Pointer to an already created CharacterSequence structure
  \return Error
  \sa PoseSequence
 */
READ_ERRORS XMLHumanPoseRead(const char* file, ObservedSequence* S);

#endif /* XMLHUMANPOSEREAD_H_ */
