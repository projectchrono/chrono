#ifndef CHAPIFEM_H
#define CHAPIFEM_H

//////////////////////////////////////////////////
//
//   ChApiFEM.h
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

#include "core/ChPlatform.h"

// Chrono::Engine version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION_UNIT_FEM 0x00000001

// When compiling this library, remember to define CH_API_COMPILE_UNIT_FEM
// (so that the symbols with 'ChApiFem' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UNIT_FEM)
	#define ChApiFem ChApiEXPORT
#else
	#define ChApiFem ChApiINPORT
#endif

#endif  // END of header