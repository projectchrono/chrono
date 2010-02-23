///////////////////////////////////////////////////
//
//   ChJs_marker.cpp
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
#include "ChJs_funct.h"
#include "ChJs_marker.h"
#include "physics/ChMarker.h"
#include "physics/ChBody.h"
#include "physics/ChGlobal.h"

namespace chrono
{



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   MARKER class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Marker_props[] = {
    {"body",       1,	JSPROP_ENUMERATE},
    {"motion_x",   2,	JSPROP_ENUMERATE},
    {"motion_y",   3,	JSPROP_ENUMERATE},
    {"motion_z",   4,	JSPROP_ENUMERATE},
    {"motion_ang", 5,	JSPROP_ENUMERATE},
    {"motion_axis",6,	JSPROP_ENUMERATE},
    {"p",		   7,	JSPROP_ENUMERATE},
    {"p_dt",       8,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"p_dtdt",     9,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"relp",	   10,	JSPROP_ENUMERATE},
    {"relp_dt",    11,	JSPROP_ENUMERATE},
    {"relp_dtdt",  12,	JSPROP_ENUMERATE},
    {"restp",	   13,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(Marker_get, ChMarker*)
	GET_JS_PROP (1,  chjs_from_data(cx,vp,this_data->GetBody(), &chjs_RBody) )
	GET_JS_PROP (2,  chjs_from_data(cx,vp,this_data->GetMotion_X(), chjs_cast_funct(this_data->GetMotion_X() )) )
	GET_JS_PROP (3,  chjs_from_data(cx,vp,this_data->GetMotion_Y(), chjs_cast_funct(this_data->GetMotion_Y() )) )
	GET_JS_PROP (4,  chjs_from_data(cx,vp,this_data->GetMotion_Z(), chjs_cast_funct(this_data->GetMotion_Z() )) )
	GET_JS_PROP (5,  chjs_from_data(cx,vp,this_data->GetMotion_ang(), chjs_cast_funct(this_data->GetMotion_ang() )) )
	GET_JS_PROP (6,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetMotion_axis()),&chjs_Vector ) )
	GET_JS_PROP (7,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetAbsCoord()), &chjs_Coordsys)  )
	GET_JS_PROP (8,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetAbsCoord_dt()), &chjs_Coordsys)  )
	GET_JS_PROP (9,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetAbsCoord_dtdt()), &chjs_Coordsys)  )
	GET_JS_PROP (10, chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetCoord()), &chjs_Coordsys)  )
	GET_JS_PROP (11, chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetCoord_dt()), &chjs_Coordsys)  )
	GET_JS_PROP (12, chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetCoord_dtdt()), &chjs_Coordsys)  )
	GET_JS_PROP (13, chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetRest_Coord()), &chjs_Coordsys)  )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Marker_set, ChMarker*)
	SET_JS_PROP (1,	&chjs_RBody, this_data->SetBody((ChBody*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (2,	&chjs_Function, this_data->SetMotion_X((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (3,	&chjs_Function, this_data->SetMotion_Y((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (4,	&chjs_Function, this_data->SetMotion_Z((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (5,	&chjs_Function, this_data->SetMotion_ang((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (6,	&chjs_Vector,   this_data->SetMotion_axis(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (7,	&chjs_Coordsys, this_data->Impose_Abs_Coord(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (10,&chjs_Coordsys, this_data->Impose_Rel_Coord(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (11,&chjs_Coordsys, this_data->SetCoord_dt(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (12,&chjs_Coordsys, this_data->SetCoord_dtdt(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (13,&chjs_Coordsys, this_data->Impose_Rel_Coord(*(Coordsys*)chjs_to_data(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsUpdate, ChMarker*, 0)
  this_data->UpdateState();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND




// ------- Method list -------------------------------
//
static JSFunctionSpec Marker_methods[] = {
	{"Update",			jsUpdate,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Marker = {
    "Marker", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Marker_get,		  Marker_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

JSClass* achjs_Marker() {return &chjs_Marker;}



////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_Marker(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_Marker,			// this class
				NULL, 0,				// constructor fx and parameters
				Marker_props, Marker_methods,
				NULL, NULL);
	return ret;
}



} // END_OF_NAMESPACE____
