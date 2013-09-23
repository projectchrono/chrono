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
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "jsapi.h"


namespace chrono 
{

void __InitChronoJavaClasses(JSContext* cx, JSObject*  glob);

	
} // END_OF_NAMESPACE____


#endif
