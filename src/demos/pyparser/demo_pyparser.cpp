///////////////////////////////////////////////////
//
//   Demo code about  
// 
//     - using the unit_PYPARSER for executing 
//       some Python program or formula
//     - using the unit_PYPARSER for loading a
//       .py scene description saved from the 
//       SolidWorks add-in
//
//   
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//  
// ------------------------------------------------ 
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------ 
///////////////////////////////////////////////////



#include "physics/ChApidll.h"
#include "unit_PYTHON/ChPython.h"
#include "physics/ChSystem.h"
#include <iostream>
#include <sstream>



using namespace chrono;

 
 

int main(int argc, char* argv[])
{
	// The DLL_CreateGlobals() - DLL_DeleteGlobals(); pair is needed if
	// global functions are needed.
	DLL_CreateGlobals(); 

	
	// Use a ChPythonEngine object.
	// Note: currently no multiple ChPythonEngine at a single time can be used.
	// Note: if you want to reset the Python environment and restart from scratch,
	//  simply use new .. delete... etc. to allocate ChPythonEngine objs on heap

	ChPythonEngine my_python;

	//
	// TEST 1   -   execute simple instructions
	//

	my_python.Run("a =12");
	my_python.Run("print(a/2)");



	// Remember this at the end of the program, if you started
	// with DLL_CreateGlobals();
	DLL_DeleteGlobals();

	return 0;
}


