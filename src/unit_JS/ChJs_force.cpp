///////////////////////////////////////////////////
//  
//   ChJs_force.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChJs_utils.h"
#include "ChJs_math.h"
#include "ChJs_system.h"
#include "ChJs_body.h"
#include "ChJs_force.h"
#include "physics/ChBody.h"
#include "physics/ChForce.h"
#include "physics/ChGlobal.h"

namespace chrono 
{


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FORCE class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Force_props[] = {
    {"body",       1,	JSPROP_ENUMERATE},
    {"mode",       2,	JSPROP_ENUMERATE},
    {"align",      3,	JSPROP_ENUMERATE},
    {"frame",      4,	JSPROP_ENUMERATE},
    {"point",      5,	JSPROP_ENUMERATE},
    {"relpoint",   6,	JSPROP_ENUMERATE},
    {"dir",		   7,	JSPROP_ENUMERATE},
    {"reldir",	   8,	JSPROP_ENUMERATE},
    {"f",		   9,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(Force_get, ChForce*)
	GET_JS_PROP (1,  chjs_from_data(cx,vp,this_data->GetBody(), &chjs_RBody) )
	GET_JS_PROP (2,  chjs_from_int(cx,vp,this_data->GetMode()) )
	GET_JS_PROP (3,  chjs_from_int(cx,vp,this_data->GetAlign()) )
	GET_JS_PROP (4,  chjs_from_int(cx,vp,this_data->GetFrame()) )
	GET_JS_PROP (5,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetVpoint()), &chjs_Vector)  )
	GET_JS_PROP (6,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetVrelpoint()), &chjs_Vector)  )
	GET_JS_PROP (7,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetDir()), &chjs_Vector)  )
	GET_JS_PROP (8,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetRelDir()), &chjs_Vector)  )
	GET_JS_PROP (9,  chjs_from_double(cx,vp,this_data->GetMforce()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Force_set, ChForce*)
	SET_JS_PROP (1,	&chjs_RBody, this_data->SetBody((ChBody*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (2,	&chjs_int, this_data->SetMode(chjs_to_int(cx,vp)) )
	SET_JS_PROP (3,	&chjs_int, this_data->SetAlign(chjs_to_int(cx,vp)) )
	SET_JS_PROP (4,	&chjs_int, this_data->SetFrame(chjs_to_int(cx,vp)) )
	SET_JS_PROP (5,	&chjs_Vector, this_data->SetVpoint(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (6,	&chjs_Vector, this_data->SetVrelpoint(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (7,	&chjs_Vector, this_data->SetDir(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (8,	&chjs_Vector, this_data->SetRelDir(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (9,	&chjs_double, this_data->SetMforce(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END


 
////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsUpdate, ChForce*, 0)
  this_data->UpdateState();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND




// ------- Method list -------------------------------
//
static JSFunctionSpec Force_methods[] = {
	{"Update",			jsUpdate,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Force = {
    "Force", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Force_get,		  Force_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

JSClass* achjs_Force() {return &chjs_Force;}


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Force(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Force,			// this class 
				NULL, 0,				// constructor fx and parameters
				Force_props, Force_methods, 
				NULL, NULL);
	return ret;
}



} // END_OF_NAMESPACE____

