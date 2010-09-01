#ifndef CHCONTROLSJS_H
#define CHCONTROLSJS_H

//////////////////////////////////////////////////
//  
//   ChControlsJS.h
//
//   Class for controls with javascript funtionality
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

#include "physics/ChControls.h"
#include "physics/ChGlobal.h"


// forward reference
struct JSScript;

namespace chrono 
{



///
/// Class for 'controls' with Javascript functionality. These objects 
/// are usually implemented in a 3d plugin, for man-in-the-loop control, 
/// or to change parameters during the simulation using scripts, etc.
///


class ChControlsJS : public ChControls 
{
								// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChControlsJS,ChControls);


		// Javascript scripts 
		// Store the filename of the script and the pointer to the compiled JSScript 

	JSScript* jsForStart;		// this javascript (from a .js file) is executed when simulation starts. 
	char jsForStartFile[200];

	JSScript* jsForUpdate;		// this javascript (from a .js file) is executed for each Update step. 
	char jsForUpdateFile[200];

	JSScript* jsForStep;		// this javascript (from a .js file) is executed for each integration step
	char jsForStepFile[200];

	JSScript* jsFor3DStep;		// this javascript (from a .js file) is executed for each 3d interface macro step 
	char jsFor3DStepFile[200];

		// File name of the window (saved from the environment of r3d, in case of plugin) which 
		// contains controls.
	char panel_window[200];
	
	char js_here_identifier[200]; // name of the JS object 'here' which can be used to contain local variables
	

public:

	ChControlsJS();
	~ChControlsJS();
	
	void Copy(ChControlsJS* source);

			//
			// FUNCTIONS
			//

	char* GetJsForStartFile() {return jsForStartFile;}
	char* GetJsForUpdateFile() {return jsForUpdateFile;}
	char* GetJsForStepFile() {return jsForStepFile;}
	char* GetJsFor3DStepFile() {return jsFor3DStepFile;}
	char* GetPanelWindowFile() {return panel_window;}
	int SetJsForStartFile(char* mfile);
	int SetJsForUpdateFile(char* mfile);
	int SetJsForStepFile(char* mfile);
	int SetJsFor3DStepFile(char* mfile);
	int SetPanelWindowFile(char* mfile);

	int SetHereVariable();

	int ExecuteJsForStart();
	int ExecuteJsForUpdate();
	int ExecuteJsForStep();
	int ExecuteJsFor3DStep();


			//
			//  - interface -
			//

	virtual int ExecuteForStart() {return ExecuteJsForStart();}
	virtual int ExecuteForUpdate() {return ExecuteJsForUpdate();}
	virtual int ExecuteForStep() {return ExecuteJsForStep();}
	virtual int ExecuteFor3DStep() {return ExecuteJsFor3DStep();}

			//
			// STREAMING
			//

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);


};
		

} // END_OF_NAMESPACE____


#endif  // END of header
