#ifndef CHGLOBALJS_H
#define CHGLOBALJS_H

//////////////////////////////////////////////////
//  
//   ChGlobalJS.h
//
//   Global javascript engine
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "core/ChLog.h"
#include "unit_JS/ChApiJS.h"


namespace chrono 
{

 class ChJavascriptEngine; // Forward reference: do not need to include'jsapi.h'
						   // header from SpiderMonkey javascript engine.


//////////////////////////////////////////////////
/// Class for global Javascript engine, for easy
/// access in all parts of the code 


class ChApiJS ChGlobalsJS
{
private:

	ChLog* scripting_log;

	int int_identifier;

public:
	ChGlobalsJS();
	~ChGlobalsJS();

			/// The logging for output of scripting languages. Never returns null, because
			/// returns by default at least a ChLogConsole() if none provided.
	ChLog& GetScriptingLog();

			/// Sets the logging for output of scripting language. 
			/// This logger won't be deleted on ChGlobals deletion.
	void   SetScriptingLog(ChLog& mlog) {scripting_log = &mlog;};

			/// Use this if you want to use default ChLogConsole(). You may
			/// need to delete previous logger.
	void   SetDefaultScriptingLog() {scripting_log = NULL;}


			// JAVASCRIPT

			/// The Javascript engine, which is able to parse js syntax wrapping
			/// lot of Chrono classes and functions. This engine is started
			/// as soon as the ChGlobal object is created, and it is stopped when
			/// the ChGlobal is destroyed. 
	ChJavascriptEngine* chjsEngine;

};



//////////////////////////////////////////////////////////////////////////

///
///  DECLARE THAT THERE'S A GLOBAL POINTER TO A "GLOBAL_VarsJS" OBJECT 
/// 

extern ChGlobalsJS* GLOBAL_VarsJS;	//***DEPRECATED*** , rather use the following..

	/// Global function to get the current ChGlobalsJS object
	/// (this is an easy way to access a single instance of globally visible
	/// data)
ChApiJS
ChGlobalsJS& CHGLOBALS_JS();

	/// Create an istance of ChGlobalsJS, then use this function at the beginning 
	/// of your program to set it as a globally visible data. 
	/// So that following calls to CHGLOBALS_JS() will always return it.
	/// If setting 0, the default static globals will be returned by CHGLOBALS_JS().
ChApiJS
void SetCHGLOBALS_JS(ChGlobalsJS* my_globals);



//
//////////////////////////////////////////////////////////////////////////







} // END_OF_NAMESPACE____

#endif
