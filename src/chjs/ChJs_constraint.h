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
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
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
