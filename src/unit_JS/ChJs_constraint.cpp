///////////////////////////////////////////////////
//  
//   ChJs_constraint.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChJs_utils.h"
#include "ChJs_math.h"
#include "ChJs_funct.h"
#include "physics/ChGlobal.h"
#include "physics/ChConstraint.h"


namespace chrono 
{



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   CONSTRAINT class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Constraint_props[] = {
    {"Cn",      0,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"C",       1,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"active",  2,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"valid",	3,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"disabled",4,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(Constraint_get, ChConstraint*)
	GET_JS_PROP (0,  chjs_from_int(cx,vp, this_data->Get_Cn())  )
	GET_JS_PROP (1,  if (this_data->IsValid())chjs_from_data(cx,vp, this_data->Get_C(), &chjs_Matrix); else chjs_from_bool(cx,vp, false); )
	GET_JS_PROP (2,  chjs_from_bool(cx,vp, this_data->IsActive())  )
	GET_JS_PROP (3,  chjs_from_bool(cx,vp, this_data->IsValid())  )
	GET_JS_PROP (4,  chjs_from_bool(cx,vp, this_data->IsDisabled())  )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Constraint_set, ChConstraint*)
	SET_JS_PROP (4,	&chjs_boolean, this_data->SetDisabled(chjs_to_bool(cx,vp)) )
SET_JS_PARSE_END


 
////////////////////////////////////////////////////////////////////
//
// METHODS
//


DEF_JS_BUILDER (Constraint_construct, ChConstraint*)
		this_data = new ChConstraint();
DEF_JS_BUILDEND

ChJS_FINALIZER (Constraint_finalize, ChConstraint*)

DEF_JS_FUNCTION(jsChConstraint_Update, ChConstraint*, 0)
  this_data->Update();
DEF_JS_FUNEND


// ------- Method list -------------------------------
//
static JSFunctionSpec Constraint_methods[] = {
	{"Update",		jsChConstraint_Update,		3},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Constraint = {
    "ChConstraint", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Constraint_get,		  Constraint_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		  Constraint_finalize,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Constraint(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Constraint,			// this class 
				Constraint_construct, 0,				// constructor fx and parameters
				Constraint_props, Constraint_methods, 
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   CONSTRAINT ChF class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Constraint_Chf_props[] = {
    {"root_function",    0,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"target_function",  1,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {0}
};

GET_JS_PARSE_BEGIN(Constraint_Chf_get, ChConstraint_Chf*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp, this_data->Get_root_function(), chjs_cast_funct(this_data->Get_root_function())  )  )
	GET_JS_PROP (1,  chjs_from_data(cx,vp, this_data->Get_target_function(), chjs_cast_funct(this_data->Get_target_function())  )  )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Constraint_Chf_set, ChConstraint_Chf*)
SET_JS_PARSE_END


 
////////////////////////////////////////////////////////////////////
//
// METHODS
//


DEF_JS_BUILDER (Constraint_Chf_construct, ChConstraint_Chf*)
	if (argc==2)
		this_data = new ChConstraint_Chf((ChFunction*)chjs_to_data(cx, argv+0), chjs_to_string(cx, argv+1));
	else
		this_data = new ChConstraint_Chf();
DEF_JS_BUILDEND

ChJS_FINALIZER (Constraint_Chf_finalize, ChConstraint_Chf*)




// ------- Method list -------------------------------
//
static JSFunctionSpec Constraint_Chf_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Constraint_Chf = {
    "ChConstraint_Chf", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Constraint_Chf_get,		  Constraint_Chf_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		  Constraint_Chf_finalize,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Constraint_Chf(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Constraint_Chf,			// this class 
				Constraint_Chf_construct, 0,				// constructor fx and parameters
				Constraint_Chf_props, Constraint_Chf_methods, 
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   CONSTRAINT ChF VAL class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Constraint_Chf_Val_props[] = {
    {"T",    0,	JSPROP_ENUMERATE},
    {"val",  1,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(Constraint_Chf_Val_get, ChConstraint_Chf_ImposeVal*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp, this_data->GetT() )  )
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->GetValue()  )  )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Constraint_Chf_Val_set, ChConstraint_Chf_ImposeVal*)
	SET_JS_PROP (0,	&chjs_double, this_data->SetT(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->SetValue(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END


 
////////////////////////////////////////////////////////////////////
//
// METHODS
//


DEF_JS_BUILDER (Constraint_Chf_Val_construct, ChConstraint_Chf_ImposeVal*)
	if (argc==4)
		this_data = new ChConstraint_Chf_ImposeVal(
				(ChFunction*)chjs_to_data(cx, argv+0), 
				chjs_to_string(cx, argv+1),
				chjs_to_double(cx, argv+2),
				chjs_to_double(cx, argv+3));
	else
		this_data = new ChConstraint_Chf_ImposeVal();
DEF_JS_BUILDEND

ChJS_FINALIZER (Constraint_Chf_Val_finalize, ChConstraint_Chf_ImposeVal*)




// ------- Method list -------------------------------
//
static JSFunctionSpec Constraint_Chf_Val_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Constraint_Chf_Val = {
    "ChConstraint_Chf_Val", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Constraint_Chf_Val_get,		  Constraint_Chf_Val_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		  Constraint_Chf_Val_finalize,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Constraint_Chf_Val(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Constraint_Chf_Val,			// this class 
				Constraint_Chf_Val_construct, 0,				// constructor fx and parameters
				Constraint_Chf_Val_props, Constraint_Chf_Val_methods, 
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   CONSTRAINT ChF CONTINUITY class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Constraint_Chf_Continuity_props[] = {
    {"continuity_order",    0,	JSPROP_ENUMERATE},
    {"interface_num",		1,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(Constraint_Chf_Continuity_get, ChConstraint_Chf_Continuity*)
	GET_JS_PROP (0,  chjs_from_int(cx,vp, this_data->GetContinuityOrder() )  )
	GET_JS_PROP (1,  chjs_from_int(cx,vp, this_data->GetInterfaceNum()  )  )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Constraint_Chf_Continuity_set, ChConstraint_Chf_Continuity*)
	SET_JS_PROP (0,	&chjs_int, this_data->SetContinuityOrder(chjs_to_int(cx,vp)) )
	SET_JS_PROP (1,	&chjs_int, this_data->SetInterfaceNum(chjs_to_int(cx,vp)) )
SET_JS_PARSE_END


 
////////////////////////////////////////////////////////////////////
//
// METHODS
//


DEF_JS_BUILDER (Constraint_Chf_Continuity_construct, ChConstraint_Chf_Continuity*)
	if (argc==4)
		this_data = new ChConstraint_Chf_Continuity(
				(ChFunction*)chjs_to_data(cx, argv+0), 
				chjs_to_string(cx, argv+1),
				chjs_to_int(cx, argv+2),
				chjs_to_int(cx, argv+3));
	else
		this_data = new ChConstraint_Chf_Continuity();
DEF_JS_BUILDEND

ChJS_FINALIZER (Constraint_Chf_Continuity_finalize, ChConstraint_Chf_Continuity*)




// ------- Method list -------------------------------
//
static JSFunctionSpec Constraint_Chf_Continuity_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Constraint_Chf_Continuity = {
    "ChConstraint_Chf_Continuity", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Constraint_Chf_Continuity_get,		  Constraint_Chf_Continuity_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		  Constraint_Chf_Continuity_finalize,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Constraint_Chf_Continuity(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Constraint_Chf_Continuity,			// this class 
				Constraint_Chf_Continuity_construct, 0,				// constructor fx and parameters
				Constraint_Chf_Continuity_props, Constraint_Chf_Continuity_methods, 
				NULL, NULL);
	return ret;
}



} // END_OF_NAMESPACE____

