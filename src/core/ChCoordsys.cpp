///////////////////////////////////////////////////
//
//   ChCoordsys.cpp
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
//   Math functions for:
//
//	 - COORDINATES
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChCoordsys.h"

namespace chrono
{



///////////////////////////////////////////////
////  COORDSYS  OPERATIONS

Coordsys  Force2Dcsys (Coordsys* cs)
{
	Coordsys res;
	res = *cs;
	res.pos.z = 0;
	res.rot.e1 = 0;
	res.rot.e2 = 0;
	return (res);
}









} // END_OF_NAMESPACE____

