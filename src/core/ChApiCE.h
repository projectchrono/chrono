#ifndef CHAPICE_H
#define CHAPICE_H

//////////////////////////////////////////////////
//  
//   ChApiCE.h
//
//   Base header for all headers that have symbols
//   that can be exported.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <cstddef>
#include <stddef.h>
#include "core/ChPlatform.h"


// Chrono::Engine version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION 0x00010300


// When compiling this library, remember to define CH_API_COMPILE
// (so that the symbols with 'ChApi' in front of them will be
// marked as exported). Otherwise, just do not define it if you 
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE)
	#define ChApi ChApiEXPORT
#else
	#define ChApi ChApiINPORT	
#endif




#endif  // END of header

