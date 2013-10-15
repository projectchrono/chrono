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

#ifndef CHJSLINK_H
#define CHJSLINK_H

///////////////////////////////////////////////////
//  
//   ChJs_link.h
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
#include "physics/ChLinksAll.h"

namespace chrono 
{


// CLASSES

extern JSClass chjs_Link; 
extern JSClass chjs_LinkLock; 
extern JSClass chjs_LinkSpring; 
extern JSClass chjs_LinkLinActuator; 
extern JSClass chjs_LinkScrew; 
extern JSClass chjs_LinkGear; 
extern JSClass chjs_LinkEngine; 
extern JSClass chjs_LinkWheel;
extern JSClass chjs_LinkBrake; 
extern JSClass chjs_LinkPneumaticActuator; 
extern JSClass chjs_LinkClearance; 

// Initiates the class 
// Returns created 'class object'. Object 'parent' can 
// be null if no inheritance desired, or other class if inheritance
// hierarchy must be built.
 
JSObject* ChJS_InitClass_Link(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkLock(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkSpring(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkLinActuator(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkScrew(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkGear(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkEngine(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkWheel(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkBrake(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkPneumaticActuator(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_LinkClearance(JSContext* cx, JSObject* glob, JSObject* parent);


// Passing a base Link* pointer to a chrono link object, the
// following function performs runtime downcast returning the exact JS class
// inherited from chjs_Function base class.  
JSClass* chjs_cast_link(ChLink* mylink);



} // END_OF_NAMESPACE____

#endif
