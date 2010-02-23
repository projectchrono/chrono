#ifndef CHAPIDLL_H
#define CHAPIDLL_H

///////////////////////////////////////////////////
//  
//   ChApidll.h
//
//   Functions to initialize and free the global
//   variables of the library.
//   Useful for using Chrono engine as dll
//   in third party projects (NOT FOR R3D PLUGIN!)
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
#include "physics/ChGlobal.h"

extern int DLL_TEST(int a, int b);

namespace chrono 
{


		/// Create and returns the GLOBAL_Vars object.
		/// Must be called ALWAYS at the beginning of the program using Chrono dll.
extern ChGlobals* DLL_CreateGlobals();

		/// Deletes the GLOBAL_Vars object.
		/// Must be called ALWAYS at the end of the program using Chrono dll.
extern void DLL_DeleteGlobals();



} // END_OF_NAMESPACE____


#endif
