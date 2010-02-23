///////////////////////////////////////////////////
// 
//   ChMath.cpp
//
//	 CHRONO  
//   ------
//   Multibody dinamics engine
//
//   Math functions for:
// 
//   - MATRIXES,
//
// ------------------------------------------------
// 	 Copyright 1996/2005 Alessandro Tasora 
// ------------------------------------------------
///////////////////////////////////////////////////



#include <math.h>

#include "core/ChMatrix.h"



namespace chrono
{





// alternate 3x3 matrix representation

void Chrono_to_Marray (ChMatrix33<>& mma, double marr[3][3])
{
	marr[0][0] = mma(0,0);	marr[0][1] = mma(0,1);	marr[0][2] = mma(0,2);
	marr[1][0] = mma(1,0);	marr[1][1] = mma(1,1);	marr[1][2] = mma(1,2);
	marr[2][0] = mma(2,0);	marr[2][1] = mma(2,1);	marr[2][2] = mma(2,2);
}
void Chrono_from_Marray (ChMatrix33<>& mma, double marr[3][3])
{
	mma(0,0) = marr[0][0];	mma(0,1) = marr[0][1];	mma(0,2) = marr[0][2];
	mma(1,0) = marr[1][0];	mma(1,1) = marr[1][1];	mma(1,2) = marr[1][2];
	mma(2,0) = marr[2][0];	mma(2,1) = marr[2][1];	mma(2,2) = marr[2][2];
}






} // END_OF_NAMESPACE____




//////// 

