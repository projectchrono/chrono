#ifndef CHJSOPTIMIZER_H
#define CHJSOPTIMIZER_H

///////////////////////////////////////////////////
//  
//   ChJs_optimizer.h
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

extern JSClass chjs_ChOptimizer;
extern JSClass chjs_ChLocalOptimizer; 
extern JSClass chjs_ChGeneticOptimizer;
extern JSClass chjs_ChHybridOptimizer;
 

// Initiates the class 
// Returns created 'class object'. Object 'parent' can 
// be null if no inheritance desired, or other class if inheritance
// hierarchy must be built.

JSObject* ChJS_InitClass_ChOptimizer(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_ChLocalOptimizer(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_ChGeneticOptimizer(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_ChHybridOptimizer(JSContext* cx, JSObject* glob, JSObject* parent);


} // END_OF_NAMESPACE____

#endif
