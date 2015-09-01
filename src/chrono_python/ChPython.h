//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2011-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHPYTHON_H
#define CHPYTHON_H

//////////////////////////////////////////////////
//  
//   ChPython.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "chrono_python/ChApiPyParser.h"
#include "core/ChLog.h"
#include "physics/ChSystem.h"


namespace chrono 
{


class ChApiPYPARSER ChPythonEngine
{
public:
			/// Create a Python parser: an interpreter that can parse Python
			/// programs, from a single formula up to large programs.
			/// NOTE!!!	currently only one instance at a time can be created.
	ChPythonEngine();

	~ChPythonEngine();


			/// Execute a program.
			/// If fails, it throws an exception, so it is wise to put it inside try..catch blocks.
	void Run(const char* program);


			/// Retrieve a value of an existing floating point variable. 
			/// Returns false if unsuccesfull.
	bool GetFloat(const char* variable, double& return_val);
			
			/// Set a value of a floating point variable. If a variable with the same
			/// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
	void SetFloat(const char* variable, const double val);


			/// Retrieve a value of an existing integer variable. 
			/// Returns false if unsuccesfull.
	bool GetInteger(const char* variable, int& return_val);
			
			/// Set a value of a integer variable. If a variable with the same
			/// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
	void SetInteger(const char* variable, const int val);


			/// Retrieve a value of an existing bool variable. 
			/// Returns false if unsuccesfull.
	bool GetBool(const char* variable, bool& return_val);
			
			/// Set a value of a bool variable. If a variable with the same
			/// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
	void SetBool(const char* variable, const bool val);


			/// Retrieve a value of an existing string variable. 
			/// Returns false if unsuccesfull.
	bool GetString(const char* variable, std::string& return_val);
			
			/// Set a value of a string variable. If a variable with the same
			/// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
	void SetString(const char* variable, std::string& val);


			/// Load a .py file as it is saved by the SolidWorks add-in exporter.
			/// You can pass a path too, ex "mydir/myotherdir/mysystem", but do NOT add .py
			/// at the end!
			/// That .py file, created when pressing the add-in button in SolidWorks CAD,
			/// contains a python program that creates an equivalent mechanism in Chrono::Engine:
			/// so it contains a sequce of creations of ChPhysicsItem objects (bodies, links, etc.).
			/// If you want to add these python C::E objects to your ChSystem that you created 
			/// in a C++ program, call this function: it will parse the .py textblock and add the created
			/// items to the ChSystem.
			/// If fails, it throws an exception, so it is wise to put it inside try..catch blocks.
	void ImportSolidWorksSystem(const char* solidworks_py_file, ChSystem& msystem); 

};




} // END_OF_NAMESPACE____


#endif  // END of ChPython.h 
