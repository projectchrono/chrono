//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHSCRIPTENGINE_H
#define CHSCRIPTENGINE_H

//////////////////////////////////////////////////
//  
//   ChScriptEngine.h
//
//   The generic interface for script engines
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



namespace chrono 
{

	
///
/// Generic compiled script for script engines
///

class ChScript
{
public:
	ChScript() {};
	virtual ~ChScript() {};
};



///
/// Generic interface for script engines
///

class ChScriptEngine
{
public:
	ChScriptEngine() {};
	virtual ~ChScriptEngine() {};

						// Set the reporter mode, 0=no report
	virtual void SetReporterMode(int mmode) = 0;

					// Simplified evaluation of a string, result goes in double*. 
					// Returns TRUE if success. 
	virtual int Eval(char*, double*) = 0; 
					// Evaluation of a string, the result is converted in string anyway 
					// (return string must be already allocated!).
	virtual int Eval(char*, char*) = 0;  
					// Set a variable to a double value, if any. Otherwise creates it. 
	virtual int SetVar(char*, double) =0; 	
					// If shell is open, prints something.
	virtual int Print(char*) =0;
					// Create a script object
	virtual ChScript* CreateScript() =0;
					// Loads a file and precompile it into a script 
	virtual int FileToScript(ChScript& script, char* file) =0;
					// Execute a precompiled script 
	virtual int ExecuteScript(ChScript& script) =0;

};



} // END_OF_NAMESPACE____

#endif
