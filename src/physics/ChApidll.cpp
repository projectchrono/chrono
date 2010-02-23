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
#include "chjs/ChJs_all.h"
#include "chjs/ChJs_Engine.h"
 
// external reference, implemented in ChJs_all.cpp
//extern "C" void InitChronoJavaClassesGlobalContext();


extern int DLL_TEST(int a, int b)
{
	return a+b;
}

namespace chrono 
{

 
extern ChGlobals* DLL_CreateGlobals()
{
	if (GLOBAL_Vars==0)
		GLOBAL_Vars = new ChGlobals;

	return GLOBAL_Vars;
}

extern void DLL_DeleteGlobals()
{
	if (GLOBAL_Vars)
		delete GLOBAL_Vars; 
	GLOBAL_Vars=0;
}


} // END_OF_NAMESPACE____

