///////////////////////////////////////////////////
//  
//   ChGlobal.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include <string.h> 
#include <time.h>

#include "physics/ChGlobal.h" 
#include "physics/ChBody.h" 
#include "physics/ChSystem.h" 
#include "unit_JS/ChJs_Engine.h"
 
 
namespace chrono 
{

//
//
// The global object pointer, and global functions
//

ChGlobals* GLOBAL_Vars =0;
 



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

ChGlobals& ChGLOBALS()
{
	if ( GlobalGlobals != NULL )
        return (*GlobalGlobals);
	else
	{
		static ChGlobals static_globals;
		return static_globals;
	}
}

void SetChGLOBALS(ChGlobals* my_globals)
{
	GlobalGlobals = my_globals;
}




} // END_OF_NAMESPACE____



// End
