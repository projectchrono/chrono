///////////////////////////////////////////////////
//  
//   ChJs_system.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChJs_utils.h"
#include "ChJs_controller.h"
#include "physics/ChController.h"


namespace chrono 
{


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   CONTROLLER class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec ChControllerPID_props[] = {
    {"Kp",          0,	JSPROP_ENUMERATE},
    {"Ki",          1,	JSPROP_ENUMERATE},
    {"Kd",          2,	JSPROP_ENUMERATE},
    {"In",			3,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"In_dt",		4,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"In_int",		5,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Out",			6,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Pcomp",		7,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Icomp",		8,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Dcomp",		9,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {0}
}; 

GET_JS_PARSE_BEGIN(ChControllerPID_get, ChControllerPID*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->P) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->I) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->D) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->Get_In() ) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp,this_data->Get_In_dt() ) )
	GET_JS_PROP (5,  chjs_from_double(cx,vp,this_data->Get_In_int() ) )
	GET_JS_PROP (6,  chjs_from_double(cx,vp,this_data->Get_Out() ) )
	GET_JS_PROP (7,  chjs_from_double(cx,vp,this_data->Get_Pcomp() ) )
	GET_JS_PROP (8,  chjs_from_double(cx,vp,this_data->Get_Icomp() ) )
	GET_JS_PROP (9,  chjs_from_double(cx,vp,this_data->Get_Dcomp() ) )
GET_JS_PARSE_END
   
SET_JS_PARSE_BEGIN(ChControllerPID_set, ChControllerPID*) 
	SET_JS_PROP (0,	&chjs_double, this_data->P = (chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->I = (chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,	&chjs_double, this_data->D = (chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER ( ChControllerPID_construct, ChControllerPID*) 
  this_data = new ChControllerPID;
DEF_JS_BUILDEND

ChJS_FINALIZER ( ChControllerPID_finalize, ChControllerPID*)


DEF_JS_FUNCTION(jsGet_output, ChControllerPID*, 2)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  chjs_from_double(cx,rval,this_data->Get_Out(chjs_to_double(cx, argv+0),chjs_to_double(cx, argv+1) ));
DEF_JS_FUNEND 

DEF_JS_FUNCTION(jsReset, ChControllerPID*, 0)
  this_data->Reset();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND




// ------- Method list -------------------------------
//
static JSFunctionSpec ChControllerPID_methods[] = {
	{"Get_output",		jsGet_output,		2},
	{"Reset",			jsReset,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_ChControllerPID = {
    "PID", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  ChControllerPID_get,		  ChControllerPID_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   ChControllerPID_finalize,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_ChControllerPID(JSContext* cx, JSObject* glob, JSObject* parent)
{

	JSObject* ret = JS_InitClass(cx, glob, 
				parent,							// parent prototype (parent class)
				&chjs_ChControllerPID,			// this class 
				ChControllerPID_construct, 0,	// constructor fx and parameters
				ChControllerPID_props, ChControllerPID_methods, 
				NULL, NULL);
	return ret;
}


} // END_OF_NAMESPACE____

