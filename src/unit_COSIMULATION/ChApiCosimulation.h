#ifndef CHAPICOSIMULATION_H
#define CHAPICOSIMULATION_H

//////////////////////////////////////////////////
//
//   ChApiCosimulation.h
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

#define CH_VERSION_UNIT_COSIMULATION 0x00000100

// When compiling this library, remember to define CH_API_COMPILE_UNIT_COSIMULATION
// (so that the symbols with 'ChApiCosimulation' in front of them will be
// marked as exported). Otherwise, just do not define it if you
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UNIT_COSIMULATION)
	#define ChApiCosimulation ChApiEXPORT
#else
	#define ChApiCosimulation ChApiINPORT
#endif

#endif  // END of header