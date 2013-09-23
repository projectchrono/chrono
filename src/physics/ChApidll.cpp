//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//  
//   ChApidll.cpp
//
//   Some interfaces for using Chrono engine as dll
//   in third party objects. 
//
//  
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
   
  
#include "physics/ChApidll.h"

  #include "parallel/ChOpenMP.h"


namespace chrono 
{

static ChGlobals* my_dll_globals = 0;
 

extern ChGlobals* DLL_CreateGlobals()
{
	//if (GLOBAL_Vars==0)
	//	GLOBAL_Vars = new ChGlobals;  //***OBOSLETE*** GLOBAL_Vars will disappear soon
	if (my_dll_globals==0)
	{
		my_dll_globals = new ChGlobals;
		SetChGLOBALS (my_dll_globals);
		GLOBAL_Vars = my_dll_globals;  //***OBOSLETE*** GLOBAL_Vars will disappear soon
	}

	return my_dll_globals;
}

extern void DLL_DeleteGlobals()
{
	if (my_dll_globals)
		delete my_dll_globals;
	my_dll_globals=0;
}


} // END_OF_NAMESPACE____

