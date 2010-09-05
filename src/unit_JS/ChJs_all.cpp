///////////////////////////////////////////////////
//
//   ChJs_all.cpp
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
//   Add all java classes to the global environment,
//   and setup the class hierarchies
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include "ChJs_all.h"

#include "ChJs_utils.h"
#include "ChJs_math.h"
#include "ChJs_chobject.h"
#include "ChJs_system.h"
#include "ChJs_body.h"
#include "ChJs_marker.h"
#include "ChJs_force.h"
#include "ChJs_link.h"
#include "ChJs_funct.h"
#include "ChJs_optimizer.h"
#include "ChJs_geometry.h"
#include "ChJs_controller.h"
#include "ChJs_impact.h"
#include "ChJs_constraint.h"
#include "ChJs_Engine.h"
#include "ChGlobalJS.h"

#include "physics/ChGlobal.h"
#include "jsapi.h"







namespace chrono
{

void __InitChronoJavaClasses(JSContext* cx, JSObject*  glob)
{

	// First, add the global stuff and utilities (like the fake classes
	// for int, double, etc.)
	ChJS_InitClasses_Utils(cx, glob);

	// Add all other classes
	//
	// NOTE! All times you wrap a new C++ class, you must add here a line
	//       into this InitChronoJavaClasses() function, so that your JS class
	//       will be initialized, and its parent class (if any) will be set, to
	//       create class hierarchy... Ex:
	//
	//      class to initialize = ChJS_Init_MyClass(context, global_object, parent_class)
	//
	JSObject* jsclass_Vector    = ChJS_InitClass_Vector(cx, glob, NULL);
	JSObject* jsclass_Quaternion= ChJS_InitClass_Quaternion(cx, glob, NULL);
	JSObject* jsclass_Coordsys  = ChJS_InitClass_Coordsys(cx, glob, NULL);
	JSObject* jsclass_Matrix    = ChJS_InitClass_Matrix(cx, glob, NULL);
	JSObject* jsclass_Geometry  = ChJS_InitClass_Geometry(cx, glob, NULL);
	//JSObject* jsclass_Placer    = ChJS_InitClass_Placer(cx, glob, jsclass_Geometry);
	JSObject* jsclass_ChObj     = ChJS_InitClass_ChObj(cx, glob, NULL);
	JSObject* jsclass_PSystem   = ChJS_InitClass_PSystem(cx, glob, jsclass_ChObj);
	JSObject* jsclass_RBody     = ChJS_InitClass_RBody(cx, glob, jsclass_ChObj);
 	JSObject* jsclass_Marker    = ChJS_InitClass_Marker(cx, glob, jsclass_ChObj);
 	JSObject* jsclass_Force     = ChJS_InitClass_Force(cx, glob, jsclass_ChObj);
 	JSObject* jsclass_Link      = ChJS_InitClass_Link(cx, glob, jsclass_ChObj);
 	JSObject* jsclass_LinkSpring= ChJS_InitClass_LinkSpring(cx, glob, jsclass_Link);
 	JSObject* jsclass_LinkLock  = ChJS_InitClass_LinkLock(cx, glob, jsclass_Link);
 	JSObject* jsclass_LinkLinActuator= ChJS_InitClass_LinkLinActuator(cx, glob, jsclass_LinkLock);
	JSObject* jsclass_LinkPneumaticActuator= ChJS_InitClass_LinkPneumaticActuator(cx, glob, jsclass_LinkLock);
 	JSObject* jsclass_LinkScrew = ChJS_InitClass_LinkScrew(cx, glob, jsclass_LinkLock);
 	JSObject* jsclass_LinkGear  = ChJS_InitClass_LinkGear(cx, glob, jsclass_LinkLock);
 	JSObject* jsclass_LinkEngine= ChJS_InitClass_LinkEngine(cx, glob, jsclass_LinkLock);
 	JSObject* jsclass_LinkWheel = ChJS_InitClass_LinkWheel(cx, glob, jsclass_LinkLock);
 	JSObject* jsclass_LinkBrake = ChJS_InitClass_LinkBrake(cx, glob, jsclass_LinkLock);
 	JSObject* jsclass_LinkClearance = ChJS_InitClass_LinkClearance(cx, glob, jsclass_LinkLock);
 	JSObject* jsclass_Function  = ChJS_InitClass_Function(cx, glob, NULL);
 	JSObject* jsclass_FunctionRamp     = ChJS_InitClass_FunctionRamp(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionSine     = ChJS_InitClass_FunctionSine(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionSigma    = ChJS_InitClass_FunctionSigma(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionPoly     = ChJS_InitClass_FunctionPoly(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionConstAcc = ChJS_InitClass_FunctionConstAcc(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionPoly345  = ChJS_InitClass_FunctionPoly345(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionFillet3  = ChJS_InitClass_FunctionFillet3(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionOperation= ChJS_InitClass_FunctionOperation(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionRecorder = ChJS_InitClass_FunctionRecorder(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionNode	   = ChJS_InitClass_FunctionNode(cx, glob, NULL);
	JSObject* jsclass_FunctionSequence = ChJS_InitClass_FunctionSequence(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionJscript  = ChJS_InitClass_FunctionJscript(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionDerive   = ChJS_InitClass_FunctionDerive(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionIntegrate= ChJS_InitClass_FunctionIntegrate(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionMirror   = ChJS_InitClass_FunctionMirror(cx, glob, jsclass_Function);
	JSObject* jsclass_FunctionRepeat   = ChJS_InitClass_FunctionRepeat(cx, glob, jsclass_Function);
 	//JSObject* jsclass_ChOptimizer			= ChJS_InitClass_ChOptimizer(cx, glob, jsclass_ChObj);
	//JSObject* jsclass_ChLocalOptimizer		= ChJS_InitClass_ChLocalOptimizer(cx, glob, jsclass_ChOptimizer);
	//JSObject* jsclass_ChGeneticOptimizer	= ChJS_InitClass_ChGeneticOptimizer(cx, glob, jsclass_ChOptimizer);
	//JSObject* jsclass_ChHybridOptimizer		= ChJS_InitClass_ChHybridOptimizer(cx, glob, jsclass_ChOptimizer);
 	JSObject* jsclass_ChControllerPID		= ChJS_InitClass_ChControllerPID(cx, glob, jsclass_ChObj);
	JSObject* jsclass_Impact		   = ChJS_InitClass_Impact(cx, glob, NULL);
	JSObject* jsclass_Constraint		   = ChJS_InitClass_Constraint(cx, glob, NULL);
	JSObject* jsclass_Constraint_Chf	   = ChJS_InitClass_Constraint_Chf(cx, glob, jsclass_Constraint);
	JSObject* jsclass_Constraint_Chf_Val   = ChJS_InitClass_Constraint_Chf_Val(cx, glob, jsclass_Constraint_Chf);
	JSObject* jsclass_Constraint_Chf_Continuity  = ChJS_InitClass_Constraint_Chf_Continuity(cx, glob, jsclass_Constraint_Chf);


	// ***TO CONTINUE..***
}


} // END_OF_NAMESPACE____



// This function can be called, from C or C++, to set up all java
// classes and default objects, for Chrono environment.
// Note: this function must be called only AFTER the
// creation of global variable object GLOBAL_Vars, which
// is used to get the global js context etc. In plugin.c,
// this is automatically done by CreateGlobals()

/*
extern "C" void InitChronoJavaClasses(JSContext* cx, JSObject*  glob)
{
	chrono::__InitChronoJavaClasses(cx, glob);
}

extern "C" void InitChronoJavaClassesGlobalContext()
{
	chrono::__InitChronoJavaClasses(chrono::CHGLOBALS().chjsEngine->cx, chrono::CHGLOBALS().chjsEngine->jglobalObj);
}
*/

