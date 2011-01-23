#ifndef CHAPIGPU_H
#define CHAPIGPU_H

//////////////////////////////////////////////////
//  
//   ChApiGPU.h 
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

//#pragma warning(disable: 4251)

#include "core/ChPlatform.h"


// Chrono::Engine unit GPU, version
//
// This is an integer, as 0xaabbccdd where
// for example version 1.2.0 is 0x00010200

#define CH_VERSION_UNIT_GPU 0x00010200


// When compiling this library, remember to define CH_API_COMPILE_UNIT_GPU
// (so that the symbols with 'ChApiGPU' in front of them will be
// marked as exported). Otherwise, just do not define it if you 
// link the library to your code, and the symbols will be imported.

#if defined(CH_API_COMPILE_UNIT_GPU)
	#define ChApiGPU ChApiEXPORT
#else
	#define ChApiGPU ChApiINPORT	
#endif




#endif  // END of header