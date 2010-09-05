///////////////////////////////////////////////////
//
//   ChJs_chobject.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChJs_utils.h"
#include "ChJs_math.h"
#include "physics/ChObject.h"
#include "physics/ChGlobal.h"


namespace chrono
{


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   OBJECT class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec ChObj_props[] = {
//    {"name",       0,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(ChObj_get, ChObj*)
	GET_JS_PROP (0, chjs_from_string(cx,vp,this_data->GetName()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(ChObj_set, ChObj*)
	SET_JS_PROP (0,	&chjs_string, this_data->SetName(chjs_to_string(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

/*
DEF_JS_FUNCTION(jsUpdate, ChObj*, 0)
  this_data->UpdateState();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND
*/



// ------- Method list -------------------------------
//
static JSFunctionSpec ChObj_methods[] = {
	//{"Update",			jsUpdate,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_ChObj = {
    "ChObj", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  ChObj_get,		  ChObj_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_ChObj(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_ChObj,			// this class
				NULL, 0,				// constructor fx and parameters
				ChObj_props, ChObj_methods,
				NULL, NULL);
	return ret;
}



} // END_OF_NAMESPACE____
