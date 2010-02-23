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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <time.h>
#include <math.h>

#include "core/ChLog.h"



namespace chrono 
{

 class ChJavascriptEngine; // Forward reference: do not need to include'jsapi.h'
						   // header from SpiderMonkey javascript engine.


//////////////////////////////////////////////////
/// Class for global CHRONO data, such as 
/// serial number, static variables, static functions, 
/// timers, etc.
/// To be documented better...


class ChGlobals
{
private:

	ChLog* scripting_log;

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
///  DECLARE THAT THERE'S A GLOBAL POINTER TO A "GLOBAL_Vars" OBJECT!!!
///  (this global object will be created, for example, at plugin loading,
///  or as soon as a main() has begun. The pointer will be set there!
///  Using a DLL, you can create global vars and initialize them by using
///  the shortcut DLL_CreateGlobals(), for example, in ChApidll.h.
///  

extern ChGlobals* GLOBAL_Vars;

//
//////////////////////////////////////////////////////////////////////////







} // END_OF_NAMESPACE____

#endif
