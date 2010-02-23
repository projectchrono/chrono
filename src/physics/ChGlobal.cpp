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
#include "chjs/ChJs_Engine.h"
 
 
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

						// by default, scripting language outputs to cout
	scripting_log = NULL;

	chjsEngine = new ChJavascriptEngine;
}



ChGlobals::~ChGlobals() 
{

	if (chjsEngine)
		delete chjsEngine;
	chjsEngine=0;
	
};




ChLog& ChGlobals::GetScriptingLog()
{
	if ( this->scripting_log != NULL )
	{
        return (*this->scripting_log);	// use custom logger or...
	}
	else
	{
		return GetLog();				// use default logger 
	}
}







} // END_OF_NAMESPACE____



// End
