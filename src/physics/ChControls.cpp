///////////////////////////////////////////////////
//
//   ChControls.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

 
  
#include <math.h>

#include "physics/ChControls.h"
#include "chjs/ChJs_Engine.h"
  
namespace chrono 
{



/////////////////////////////////////////////////////////
/// 
///   CLASS
///
///


// Register into the object factory, to enable run-time
// dynamic creation and persistence 
ChClassRegister<ChControls> a_registration_ChControls;



// CREATE

ChControls::ChControls()
{
	this->SetIdentifier(GLOBAL_Vars->GetUniqueIntID()); // mark with unique ID

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
	sprintf (js_here_identifier, "__h%d", GLOBAL_Vars->GetUniqueIntID());
	sprintf (mcommandbuffer, "%s= new Object;", js_here_identifier);
	GLOBAL_Vars->chjsEngine->chjs_Eval(mcommandbuffer, &foo);
}

// DESTROY

ChControls::~ChControls()
{
	if (jsForStart)	 JS_DestroyScript(GLOBAL_Vars->chjsEngine->cx, jsForStart);
	if (jsForUpdate) JS_DestroyScript(GLOBAL_Vars->chjsEngine->cx, jsForUpdate);
	if (jsForStep)   JS_DestroyScript(GLOBAL_Vars->chjsEngine->cx, jsForStep);
	if (jsFor3DStep) JS_DestroyScript(GLOBAL_Vars->chjsEngine->cx, jsFor3DStep);
	jsForStart = NULL;
	jsForUpdate = NULL;
	jsForStep = NULL;
	jsFor3DStep = NULL;

	// delete the JS unique object
	char mcommandbuffer [280];
	double foo;
	sprintf (mcommandbuffer, "delete (%s);", js_here_identifier);
	GLOBAL_Vars->chjsEngine->chjs_Eval(mcommandbuffer, &foo);
}
 

void ChControls::Copy(ChControls* source)
{
	// First copy the parent class data...
	ChObj::Copy(source);
	
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
	GLOBAL_Vars->chjsEngine->chjs_Eval(mcommandbuffer, &foo);
	sprintf (js_here_identifier, "__h%d", GLOBAL_Vars->GetUniqueIntID() );
	sprintf (mcommandbuffer, "%s=clonate(%s);", js_here_identifier, source->js_here_identifier);
	GLOBAL_Vars->chjsEngine->chjs_Eval(mcommandbuffer, &foo);
}


//
// Set/Get routines
//

int ChControls::SetJsForStartFile(char* mfile)
{
	strcpy (this->jsForStartFile, mfile);
	return GLOBAL_Vars->chjsEngine->chjs_FileToScript(&this->jsForStart, mfile);
}

int ChControls::SetJsForUpdateFile(char* mfile)
{
	strcpy (this->jsForUpdateFile, mfile);
	return GLOBAL_Vars->chjsEngine->chjs_FileToScript(&this->jsForUpdate, mfile);
}

int ChControls::SetJsForStepFile(char* mfile)
{
	strcpy (this->jsForStepFile, mfile);
	return GLOBAL_Vars->chjsEngine->chjs_FileToScript(&this->jsForStep, mfile);
}

int ChControls::SetJsFor3DStepFile(char* mfile)
{
	strcpy (this->jsFor3DStepFile, mfile);
	return GLOBAL_Vars->chjsEngine->chjs_FileToScript(&this->jsFor3DStep, mfile);
}

int ChControls::SetPanelWindowFile(char* mfile)
{
	strcpy (this->panel_window, mfile);
	return TRUE;
}

//
// This function is called before each script execution:
// it sets the 'here' javascript pointer to the local object.
//

int ChControls::SetHereVariable()
{
	char mcommandbuffer [200];
	double foo;
	sprintf (mcommandbuffer, "here=%s;", js_here_identifier);
	GLOBAL_Vars->chjsEngine->chjs_Eval(mcommandbuffer, &foo);
	return TRUE;
}

//
// Script execution functions
//

int ChControls::ExecuteJsForStart()
{
	jsval jsresult;
	if (this->jsForStart) 	{
		SetHereVariable();
		JS_ExecuteScript(GLOBAL_Vars->chjsEngine->cx, GLOBAL_Vars->chjsEngine->jglobalObj,
						 this->jsForStart, &jsresult);
	} return true;
}

int ChControls::ExecuteJsForUpdate()
{
	jsval jsresult;
	if (this->jsForUpdate) {
		SetHereVariable();
		JS_ExecuteScript(GLOBAL_Vars->chjsEngine->cx, GLOBAL_Vars->chjsEngine->jglobalObj,
						 this->jsForUpdate, &jsresult);
	} return true;
}

int ChControls::ExecuteJsForStep()
{
	jsval jsresult;
	if (this->jsForStep) {
		SetHereVariable();
		JS_ExecuteScript(GLOBAL_Vars->chjsEngine->cx, GLOBAL_Vars->chjsEngine->jglobalObj,
						 this->jsForStep, &jsresult);
	} return true;
}

int ChControls::ExecuteJsFor3DStep()
{
	jsval jsresult;
	if (this->jsFor3DStep) {
		SetHereVariable();
		JS_ExecuteScript(GLOBAL_Vars->chjsEngine->cx, GLOBAL_Vars->chjsEngine->jglobalObj,
						 this->jsFor3DStep, &jsresult);
	} return true;
}


// 
// FILE I/O
//

void ChControls::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number 
	mstream.VersionWrite(1); 
		// serialize parent class too
	ChObj::StreamOUT(mstream);

		// stream out all member data
	mstream << GetJsForStartFile();
	mstream << GetJsForStartFile();
	mstream << GetJsForUpdateFile();
	mstream << GetJsForStepFile();
	mstream << GetJsFor3DStepFile();
	mstream << GetPanelWindowFile();
}
				
void ChControls::StreamIN(ChStreamInBinary& mstream)
{
		// class version number 
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChObj::StreamIN(mstream);

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

void ChControls::StreamOUT(ChStreamOutAscii& mstream)
{
	//***TO DO***
}




} // END_OF_NAMESPACE____ 



////// end
