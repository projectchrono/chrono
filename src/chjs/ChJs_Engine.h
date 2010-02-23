#ifndef CHJS_ENGINE_H
#define CHJS_ENGINE_H

//////////////////////////////////////////////////
//  
//   ChJs_Engine.h
//
//   The javascript parser, JS engine 
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

#include "jsapi.h"
#include "jsobj.h"
#include "jsmath.h"


namespace chrono 
{



///
/// Class for the engine which parses and executes Javascript
/// programs, which wraps Chrono function/classes.
///

class ChJavascriptEngine
{
public:
		
	JSRuntime *rt;		// points to the global JS runtime    (automatically created when creating a ChGlobal object)
	JSContext *cx;		// points to the global JS context    ( "  "  )
	JSObject *jglobalObj; // points to the global JS object	  ( "  "  )
	JSClass  jglobalClass;// the global JS class str. ( "  "  )

					// If 0, no error report, if 1, use error reporter  --internal--
	int chjs_reporter_mode;	

					// Set the reporter mode.
	void chjs_SetReporterMode(int mmode);

					// Initiates javascript runtime rt, context cx, base object jglobalObj. 
					// Return false if cannot do.
	int Init_Js_Eng();	
					// Simplified evaluation of a string, result goes in double*. 
					// Returns TRUE if success. 
	int chjs_Eval(char*, double*); 
					// Evaluation of a string, the result is converted in string anyway 
					// (return string must be already allocated!).
	int chjs_Eval(char*, char*);  
					// Set a variable to a double value, if any. Otherwise creates it. 
	int chjs_SetVar(char*, double); 	
					// If shell is open, prints something.
	int chjs_Print(char*);
					// Loads a file and compile it into a script (if passed script pointer is not null,
					// the previous pointed script is destroyed. The resulting script can be NULL if nothing could be compiled.
	int chjs_FileToScript(JSScript** script, char* file);

					// When doing chjs_Eval in some Chrono context, you may set both the following if evaluation of the js command "context()" 
					// should return the specific js object of that context (otherwise returns default jglobalObj). 
					// Please remember to setback them to NULL just after the evaluation, for "context()" to return default jglobalObj. 

	JSClass* chjs_contextclass;  // pointer to class id of returned js object in evaluating js command "context()"
	void*    chjs_contextdata;   // ..and pointer to private data of that object (ex, a pointer to a RBody or to a Link)
	

			/// Constructor. Initialize, using JS_NewRuntime() and JS_NewContext()
	ChJavascriptEngine();

			/// Destructor. Kills context and runtime.
	~ChJavascriptEngine();

};




//////////////////////////////////////////////////////////////////////////


						/* custom Javascript error reporter */
extern void chjs_errorReporter(JSContext *cx, const char *message, JSErrorReport *report);
extern void chjs_errorReporterVoid(JSContext *cx, const char *message, JSErrorReport *report);





} // END_OF_NAMESPACE____

#endif
