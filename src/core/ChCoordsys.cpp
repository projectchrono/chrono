//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2011 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

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

