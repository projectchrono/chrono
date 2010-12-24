///////////////////////////////////////////////////
//  
//   ChGlobalJS.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

  
#include <string.h> 

#include "physics/ChBody.h" 
#include "physics/ChSystem.h"
#include "unit_JS/ChGlobalJS.h" 
#include "unit_JS/ChJs_Engine.h"
 

#ifndef CH_API_COMPILE_UNIT_JS
#error Warning! You are compiling the JS JavaScript unit of Chrono::Engine, \
	so you need to define CH_API_COMPILE_UNIT_JS (add that symbol \
	to the compiler defines, for all compiled files in this unit). 
#endif 

 
namespace chrono 
{

//
//
// The global object pointer, and global functions
//

ChGlobalsJS* GLOBAL_VarsJS =0;  //***DEPRECATED***
 



// The class members

  
ChGlobalsJS::ChGlobalsJS()
{
						// by default, scripting language outputs to cout
	scripting_log = NULL;

	chjsEngine = new ChJavascriptEngine;
}



ChGlobalsJS::~ChGlobalsJS() 
{

	if (chjsEngine)
		delete chjsEngine;
	chjsEngine=0;
	
};




ChLog& ChGlobalsJS::GetScriptingLog()
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




///////////////////////////////////////////////////////


//
// The pointer to the global 'ChGlobalsJS'
//

static ChGlobalsJS*  GlobalGlobalsJS = 0 ;

ChGlobalsJS& ChGLOBALS_JS()
{
	if ( GlobalGlobalsJS != NULL )
        return (*GlobalGlobalsJS);
	else
	{
		static ChGlobalsJS static_globals_js;
		return static_globals_js;
	}
}

void SetChGLOBALS_JS(ChGlobalsJS* my_globals)
{
	GlobalGlobalsJS = my_globals;
}






} // END_OF_NAMESPACE____



// End
