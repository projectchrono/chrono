//////////////////////////////////////////////////
//  
//   ChApidll.cpp
//
//   Some interfaces for using Chrono engine as dll
//   in third party objects. 
//
//  
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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

