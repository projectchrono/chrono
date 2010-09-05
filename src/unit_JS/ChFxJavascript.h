#ifndef CHFXJAVASCRIPT_H
#define CHFXJAVASCRIPT_H

//////////////////////////////////////////////////
//
//   ChFx.h
//
//   Classes for object defining math functions of 
//   the type A=f(B) , to be used in optimization/etc.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "physics/ChFx.h"
#include "unit_JS/ChOptvar.h"
#include "unit_JS/ChGlobalJS.h"

struct JSScript;

namespace chrono
{


/// Class for A=f(B) math functions, where the function
/// is defined by a Javascript formula that returns a scalar value,
/// and input variables are set from a list of Javascript names.

class ChFxJavascript : public ChFx
{
public:
				/// Create the function wrapper.
	ChFxJavascript();
	virtual ~ChFxJavascript();

				/// Sets the objective function to maximize, as ASCII interpreted formula
				/// and also precompile it into a script for optimizing speed. 
				/// If returns 0, script has syntax error.
	virtual int SetFunction (char* mformula);
	char* GetFunction () {return function;};

				/// Adds the optimization variables (will be deleted automatically at object's end)	
	virtual void AddInVar (ChOptVar* newvar);
	virtual void RemoveInVar (ChOptVar* newvar);

	virtual ChOptVar* GetVarList() {return optvarlist;};
	virtual void SetVarList(ChOptVar* mv) {optvarlist = mv;};

			/// Return the number of added variables (unlocked)
	int GetNumOfVars();

	virtual int  CompileInVar(); 	// to speed up code..

				/// The Javascript environment gets the current state of
				/// input variables. Ret. 0 if can't set values. Mostly not needed by user.
	int Vars_to_System(double  x[]);
	int Vars_to_System(const ChMatrix<>* x);
				/// The in variables gets their corresponding values in Javascript environment.
				/// Return 0 if can't get values. Mostly not needed by user.
	int System_to_Vars(double  x[]);
	int System_to_Vars(ChMatrix<>* x);

	char* GetLastError() {return err_message;}

		/// INTERFACE: 
		/// Evaluate A=f(B) 
	virtual void Eval  (	  ChMatrix<>& A,	///< results here
						const ChMatrix<>& B		///< input here
					   );

private:
	char function[300];		// the objective formula to be maximized, as ASCII expression
	ChOptVar* optvarlist;	// list of variables to be optimized;
	JSScript* fx_script;	// JavaScript script coming from compilation of function[]   {internal}
	char err_message[200];		// the ok/warning/error messages are written here
};






} // END_OF_NAMESPACE____

#endif
