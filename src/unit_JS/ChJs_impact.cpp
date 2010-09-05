///////////////////////////////////////////////////
//  
//   ChJs_impact.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChJs_utils.h"
#include "ChJs_math.h"
#include "ChJs_body.h"
#include "ChJs_impact.h"
#include "physics/ChImpacts.h"
#include "physics/ChGlobal.h"

namespace chrono 
{



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   IMPACT class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Impact_props[] = {
    {"Iabs",       1,	JSPROP_ENUMERATE|JSPROP_READONLY},
	{"Pabs",       2,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Nabs",       3,	JSPROP_ENUMERATE|JSPROP_READONLY}, 
    {"bodyA",      4,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"bodyB",      5,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"local_coords", 6,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {0}
};

GET_JS_PARSE_BEGIN(Impact_get, ChImpact*)
	GET_JS_PROP (1,  chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_Iabs()), &chjs_Vector)  )
	GET_JS_PROP (2,  chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_Pabs()), &chjs_Vector)  )
	GET_JS_PROP (3,  chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_Nabs()), &chjs_Vector)  )
	GET_JS_PROP (4,  chjs_from_data(cx,vp,this_data->bodyA, &chjs_RBody) )	
	GET_JS_PROP (5,  chjs_from_data(cx,vp,this_data->bodyB, &chjs_RBody) )
	GET_JS_PROP (6,  chjs_from_data(cx,vp,this_data->Get_A(), &chjs_Matrix) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Impact_set, ChImpact*)

SET_JS_PARSE_END


 
////////////////////////////////////////////////////////////////////
//
// METHODS
//





// ------- Method list -------------------------------
//
static JSFunctionSpec Impact_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Impact = {
    "Ch_Impact", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Impact_get,		  Impact_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Impact(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Impact,			// this class 
				NULL, 0,				// constructor fx and parameters
				Impact_props, Impact_methods, 
				NULL, NULL);
	return ret;
}


} // END_OF_NAMESPACE____
