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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "unit_PYTHON/ChApiPyParser.h"
#include "core/ChLog.h"



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
			/// If fails, it throws an exception.
	void Run(const char* program) throw(ChException);

			/// Retrieve a value of an existing floating point variable. 
			/// Returns false if unsuccesfull.
	bool ChPythonEngine::GetFloat(const char* variable, double& return_val);
			
			/// Set a value of a floating point variable. If a variable with the same
			/// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
	void ChPythonEngine::SetFloat(const char* variable, const double val);


			/// Retrieve a value of an existing integer variable. 
			/// Returns false if unsuccesfull.
	bool ChPythonEngine::GetInteger(const char* variable, int& return_val);
			
			/// Set a value of a integer variable. If a variable with the same
			/// name is existing, it is overwritten, otherwise it is created (in __main__ namespace)
	void ChPythonEngine::SetInteger(const char* variable, const int val);

};




} // END_OF_NAMESPACE____


#endif  // END of ChPython.h 
