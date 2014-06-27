//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHGLOBAL_H
#define CHGLOBAL_H

//////////////////////////////////////////////////
//  
//   ChGlobal.h
//
//   Global variables.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <time.h>
#include <math.h>

#include "core/ChLog.h"
#include "core/ChApiCE.h"


namespace chrono 
{




//////////////////////////////////////////////////
/// Class for global CHRONO data, such as 
/// serial number, static variables, static functions, 
/// timers, etc.
/// To be documented better...


class ChApi ChGlobals
{
private:

	int int_identifier;

public:
	ChGlobals();
	~ChGlobals();

	int WriteComments;
	int WriteAllFeatures;
	int SkipNframesOutput;
	int angle_units;



			/// Time utilities (better: use ChTime.h)
	clock_t t_start, t_stop;
    double  t_duration; 
	double  t_simtime, t_geometrytime;
	void Timer_START() {t_start = clock(); t_duration= 0;}
	void Timer_STOP() {t_stop = clock(); t_duration +=  ((double)((double)(t_stop - t_start) / CLOCKS_PER_SEC));}
	void Timer_RESTART() {t_start = clock();}


			/// Return an unique identifier, of type integer. 
	int GetUniqueIntID() {int_identifier++; return int_identifier;};

};



//////////////////////////////////////////////////////////////////////////



	/// Global function to get the current ChGlobals object
	/// (this is an easy way to access a single instance of globally visible
	/// data)
ChApi 
ChGlobals& CHGLOBALS();

	/// Create an istance of ChGlobals, then use this function at the beginning 
	/// of your program to set it as a globally visible data. 
	/// So that following calls to CHGLOBALS() will always return it.
	/// If setting 0, the default static globals will be returned by CHGLOBALS().
ChApi 
void SetCHGLOBALS(ChGlobals* my_globals);


//
//////////////////////////////////////////////////////////////////////////







} // END_OF_NAMESPACE____

#endif
