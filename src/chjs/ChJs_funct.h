#ifndef CHJSFUNCT_H
#define CHJSFUNCT_H

///////////////////////////////////////////////////
//  
//   ChJs_funct.h
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
#include "physics/ChFunction.h"


namespace chrono 
{


// CLASSES

extern JSClass chjs_Function; 
extern JSClass chjs_FunctionRamp; 
extern JSClass chjs_FunctionSine; 
extern JSClass chjs_FunctionSigma; 
extern JSClass chjs_FunctionPoly; 
extern JSClass chjs_FunctionConstAcc; 
extern JSClass chjs_FunctionPoly345; 
extern JSClass chjs_FunctionFillet3; 
extern JSClass chjs_FunctionOperation; 
extern JSClass chjs_FunctionRecorder; 
extern JSClass chjs_FunctionNode;
extern JSClass chjs_FunctionSequence; 
extern JSClass chjs_FunctionJscript; 
extern JSClass chjs_FunctionMirror;
extern JSClass chjs_FunctionRepeat;
extern JSClass chjs_FunctionIntegrate;
extern JSClass chjs_FunctionDerive;


// Initiates the class 
// Returns created 'class object'. Object 'parent' can 
// be null if no inheritance desired, or other class if inheritance
// hierarchy must be built.
 
JSObject* ChJS_InitClass_Function(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionRamp(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionSine(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionSigma(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionPoly(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionConstAcc(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionPoly345(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionFillet3(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionOperation(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionRecorder(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionNode(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionSequence(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionJscript(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionMirror(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionRepeat(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionIntegrate(JSContext* cx, JSObject* glob, JSObject* parent);
JSObject* ChJS_InitClass_FunctionDerive(JSContext* cx, JSObject* glob, JSObject* parent);

// Passing a base ChFunction* pointer to a function object, the
// following function performs runtime downcast returning the exact JS class
// inherited from chjs_Function base class.  
JSClass* chjs_cast_funct(ChFunction* myfx);


} // END_OF_NAMESPACE____

#endif

