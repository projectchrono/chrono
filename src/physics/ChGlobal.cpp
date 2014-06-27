//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

///////////////////////////////////////////////////
//  
//   ChGlobal.cpp
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include <string.h> 
#include <time.h>

#include "physics/ChGlobal.h" 
#include "physics/ChBody.h" 
#include "physics/ChSystem.h" 
 
 
namespace chrono 
{

//
//
// The global functions
//

 



// The class members

  
ChGlobals::ChGlobals()
{
	WriteComments = 1;
	WriteAllFeatures = 0;

	t_duration = 0;

	SkipNframesOutput = 0; 


	time_t ltime;		// setup a safe unique first ID
	struct tm *nt;
	time (&ltime);
	nt = localtime(&ltime);
	int_identifier = (int)(3600*nt->tm_hour + 60*nt->tm_min + nt->tm_sec);

}



ChGlobals::~ChGlobals() 
{
	
};



//
// The pointer to the global 'ChGlobals'
//

static ChGlobals*  GlobalGlobals = 0 ;

ChGlobals& CHGLOBALS()
{
	if ( GlobalGlobals != NULL )
        return (*GlobalGlobals);
	else
	{
		static ChGlobals static_globals;
		return static_globals;
	}
}

void SetCHGLOBALS(ChGlobals* my_globals)
{
	GlobalGlobals = my_globals;
}




} // END_OF_NAMESPACE____



// End
