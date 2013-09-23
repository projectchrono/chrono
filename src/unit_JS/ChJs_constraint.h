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

#ifndef CHJSCONSTRAINT_H
#define CHJSCONSTRAINT_H

///////////////////////////////////////////////////
//  
//   ChJs_constraint.h
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

extern JSClass chjs_Constraint; 
extern JSClass chjs_Constraint_Chf; 
extern JSClass chjs_Constraint_Chf_Val; 
extern JSClass chjs_Constraint_Chf_Continuity;

// Initiates the class 
// Returns created 'class object'. Object 'parent' can 
// be null if no inheritance desired, or other class if inheritance
// hierarchy must be built.
 
JSObject* ChJS_InitClass_Constraint(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_Constraint_Chf(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_Constraint_Chf_Val(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_Constraint_Chf_Continuity(JSContext* cx, JSObject* glob, JSObject* parent);


} // END_OF_NAMESPACE____

#endif
