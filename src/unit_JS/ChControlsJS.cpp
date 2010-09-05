///////////////////////////////////////////////////
//
//   ChControlsJS.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
  
#include <math.h>

#include "unit_JS/ChControlsJS.h"
#include "unit_JS/ChJs_Engine.h"
#include "unit_JS/ChGlobalJS.h"

namespace chrono 
{



/////////////////////////////////////////////////////////
/// 
///   CLASS
///
///


// Register into the object factory, to enable run-time
// dynamic creation and persistence 
ChClassRegister<ChControlsJS> a_registration_ChControlsJS;



// CREATE

ChControlsJS::ChControlsJS()
{
	this->SetIdentifier(CHGLOBALS().GetUniqueIntID()); // mark with unique ID

	jsForStart = NULL;
	jsForUpdate = NULL;
	jsForStep = NULL;
	jsFor3DStep = NULL;
	
	strcpy (jsForStartFile, "");
	strcpy (jsForUpdateFile, "");
	strcpy (jsForStepFile, "");
	strcpy (jsFor3DStepFile, "");

	strcpy (panel_window, "");


	// create the JS unique object
	char mcommandbuffer [280];
	double foo;
	sprintf (js_here_identifier, "__h%d", CHGLOBALS().GetUniqueIntID());
	sprintf (mcommandbuffer, "%s= new Object;", js_here_identifier);
	CHGLOBALS_JS().chjsEngine->chjs_Eval(mcommandbuffer, &foo);
}

// DESTROY

ChControlsJS::~ChControlsJS()
{
	if (jsForStart)	 JS_DestroyScript(CHGLOBALS_JS().chjsEngine->cx, jsForStart);
	if (jsForUpdate) JS_DestroyScript(CHGLOBALS_JS().chjsEngine->cx, jsForUpdate);
	if (jsForStep)   JS_DestroyScript(CHGLOBALS_JS().chjsEngine->cx, jsForStep);
	if (jsFor3DStep) JS_DestroyScript(CHGLOBALS_JS().chjsEngine->cx, jsFor3DStep);
	jsForStart = NULL;
	jsForUpdate = NULL;
	jsForStep = NULL;
	jsFor3DStep = NULL;

	// delete the JS unique object
	char mcommandbuffer [280];
	double foo;
	sprintf (mcommandbuffer, "delete (%s);", js_here_identifier);
	CHGLOBALS_JS().chjsEngine->chjs_Eval(mcommandbuffer, &foo);
}
 

void ChControlsJS::Copy(ChControlsJS* source)
{
	// First copy the parent class data...
	//ChObj::Copy(source); nothing, was an interface
	
	// Copy other data..
	SetJsForStartFile(source->jsForStartFile);
	SetJsForUpdateFile(source->jsForUpdateFile);
	SetJsForStepFile(source->jsForStepFile);
	SetJsFor3DStepFile(source->jsFor3DStepFile);

	strcpy (panel_window, source->panel_window);

	// clonate the JS object, with different identifier
	char mcommandbuffer [280];
	double foo;
	sprintf (mcommandbuffer, "delete(%s);", js_here_identifier);
	CHGLOBALS_JS().chjsEngine->chjs_Eval(mcommandbuffer, &foo);
	sprintf (js_here_identifier, "__h%d", CHGLOBALS().GetUniqueIntID() );
	sprintf (mcommandbuffer, "%s=clonate(%s);", js_here_identifier, source->js_here_identifier);
	CHGLOBALS_JS().chjsEngine->chjs_Eval(mcommandbuffer, &foo);
}


//
// Set/Get routines
//

int ChControlsJS::SetJsForStartFile(char* mfile)
{
	strcpy (this->jsForStartFile, mfile);
	return CHGLOBALS_JS().chjsEngine->chjs_FileToScript(&this->jsForStart, mfile);
}

int ChControlsJS::SetJsForUpdateFile(char* mfile)
{
	strcpy (this->jsForUpdateFile, mfile);
	return CHGLOBALS_JS().chjsEngine->chjs_FileToScript(&this->jsForUpdate, mfile);
}

int ChControlsJS::SetJsForStepFile(char* mfile)
{
	strcpy (this->jsForStepFile, mfile);
	return CHGLOBALS_JS().chjsEngine->chjs_FileToScript(&this->jsForStep, mfile);
}

int ChControlsJS::SetJsFor3DStepFile(char* mfile)
{
	strcpy (this->jsFor3DStepFile, mfile);
	return CHGLOBALS_JS().chjsEngine->chjs_FileToScript(&this->jsFor3DStep, mfile);
}

int ChControlsJS::SetPanelWindowFile(char* mfile)
{
	strcpy (this->panel_window, mfile);
	return TRUE;
}

//
// This function is called before each script execution:
// it sets the 'here' javascript pointer to the local object.
//

int ChControlsJS::SetHereVariable()
{
	char mcommandbuffer [200];
	double foo;
	sprintf (mcommandbuffer, "here=%s;", js_here_identifier);
	CHGLOBALS_JS().chjsEngine->chjs_Eval(mcommandbuffer, &foo);
	return TRUE;
}

//
// Script execution functions
//

int ChControlsJS::ExecuteJsForStart()
{
	jsval jsresult;
	if (this->jsForStart) 	{
		SetHereVariable();
		JS_ExecuteScript(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj,
						 this->jsForStart, &jsresult);
	} return true;
}

int ChControlsJS::ExecuteJsForUpdate()
{
	jsval jsresult;
	if (this->jsForUpdate) {
		SetHereVariable();
		JS_ExecuteScript(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj,
						 this->jsForUpdate, &jsresult);
	} return true;
}

int ChControlsJS::ExecuteJsForStep()
{
	jsval jsresult;
	if (this->jsForStep) {
		SetHereVariable();
		JS_ExecuteScript(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj,
						 this->jsForStep, &jsresult);
	} return true;
}

int ChControlsJS::ExecuteJsFor3DStep()
{
	jsval jsresult;
	if (this->jsFor3DStep) {
		SetHereVariable();
		JS_ExecuteScript(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj,
						 this->jsFor3DStep, &jsresult);
	} return true;
}


// 
// FILE I/O
//

void ChControlsJS::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number 
	mstream.VersionWrite(1); 
		// serialize parent class too
	ChControls::StreamOUT(mstream);

		// stream out all member data
	mstream << GetJsForStartFile();
	mstream << GetJsForStartFile();
	mstream << GetJsForUpdateFile();
	mstream << GetJsForStepFile();
	mstream << GetJsFor3DStepFile();
	mstream << GetPanelWindowFile();
}
				
void ChControlsJS::StreamIN(ChStreamInBinary& mstream)
{
		// class version number 
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChControls::StreamIN(mstream);

		// stream in all member data
	char fpa[250];
	mstream >> fpa;
	 SetJsForStartFile(fpa);
	mstream >> fpa;
	 SetJsForStartFile(fpa);
	mstream >> fpa;
	 SetJsForUpdateFile(fpa);
	mstream >> fpa;
	 SetJsForStepFile(fpa);
	mstream >> fpa;
	 SetJsFor3DStepFile(fpa);
	mstream >> fpa;
	 SetPanelWindowFile(fpa);
}





} // END_OF_NAMESPACE____ 



////// end
