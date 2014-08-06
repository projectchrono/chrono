//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHAPIDLL_H
#define CHAPIDLL_H

///////////////////////////////////////////////////
//  
//   ChApidll.h
//
//   Functions to initialize and free the global
//   variables of the library.
//   Useful for using Chrono engine as dll
//   in third party projects (NOT FOR R3D PLUGIN!)
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>
#include "physics/ChGlobal.h"
#include "core/ChApiCE.h"


namespace chrono 
{

  extern ChApi void DLL_CreateGlobals();

  extern ChApi void DLL_DeleteGlobals();

} // END_OF_NAMESPACE____


#endif
