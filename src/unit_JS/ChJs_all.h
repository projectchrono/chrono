#ifndef CHJSALL_H
#define CHJSALL_H

///////////////////////////////////////////////////
//  
//   ChJs_all.h
//
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//
//   Add all java classes to the global environment, 
//   and setup the class hierarchies
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "jsapi.h"


namespace chrono 
{

void __InitChronoJavaClasses(JSContext* cx, JSObject*  glob);

	
} // END_OF_NAMESPACE____


#endif
