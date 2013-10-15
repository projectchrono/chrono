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

#ifndef CHJSMATH_H
#define CHJSMATH_H

///////////////////////////////////////////////////
//  
//   ChJs_math.h
//
//	 CHRONO 
//   ------
//   Multibody dinamics engine
//
//   define JS wrapper class
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "jsapi.h"

namespace chrono 
{



// CLASSES

extern JSClass chjs_Vector;
JSClass* achjs_Vector();  // get address of static 'Javascript class' structure

extern JSClass chjs_Quaternion;
JSClass* achjs_Quaternion();  // get address of static 'Javascript class' structure

extern JSClass chjs_Coordsys;
JSClass* achjs_Coordsys();  // get address of static 'Javascript class' structure

extern JSClass chjs_Matrix;
JSClass* achjs_Matrix();  // get address of static 'Javascript class' structure
 

// Initiates the class 
// Returns created 'class object'. Object 'parent' can 
// be null if no inheritance desired, or other class if inheritance
// hierarchy must be built.
 
JSObject* ChJS_InitClass_Vector(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_Quaternion(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_Coordsys(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_Matrix(JSContext* cx, JSObject* glob, JSObject* parent);


} // END_OF_NAMESPACE____

#endif
