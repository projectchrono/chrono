#ifndef CHC_COMPILE_H
#define CHC_COMPILE_H

//////////////////////////////////////////////////
//  
//   ChCCompile.h
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



namespace chrono 
{
namespace collision 
{


// prevents compiler warnings when PQP_REAL is float
/*
inline float sqrt(float x) { return (float)sqrt((double)x); }
inline float cos(float x) { return (float)cos((double)x); }
inline float sin(float x) { return (float)sin((double)x); }
inline float fabs(float x) { return (float)fabs((double)x); }
*/

	
	
//-------------------------------------------------------------------------
//
// PQP_REAL
//
// This is the floating point type used throughout PQP.  doubles are
// recommended, both for their precision and because the software has
// mainly been tested using them.  However, floats appear to be faster 
// (by 60% on some machines).
//
//-------------------------------------------------------------------------

typedef double PQP_REAL;




} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____

#endif
