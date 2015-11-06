/*
 * XMLHumanActivityRead.h
 *
 *  Created on: Aug 10, 2012
 *      Author: madisob
 */

#ifndef XMLHUMANACTIVITYREAD_H_
#define XMLHUMANACTIVITYREAD_H_

#include "XMLCommon.h"
#include "ActivityPattern.h"

//! Reads xml file and converts to an activity
/*!
  Reads an xml file which contains information for an an activity.
  The XML file is parsed and the sequence P is filled out
  \param file Filename of the XML file
  \param A Pointer to an already created Pattern structure to fill out
  \return Error
  \sa ActivityAlphabet
 */
READ_ERRORS XMLHumanActivityRead(const char* file, ActivityPattern* A);

#endif /* XMLHUMANACTIVITYREAD_H_ */
