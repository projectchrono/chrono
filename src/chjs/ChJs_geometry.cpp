///////////////////////////////////////////////////
//  
//   ChJs_geometry.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include "ChJs_utils.h"
#include "ChJs_geometry.h"
#include "ChJs_math.h"
#include "physics/ChSystem.h"
#include "physics/ChGlobal.h"
#include "physics/ChExternalObject.h"



namespace chrono 
{



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   GEOMETRY class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Geometry_props[] = {
	{"p",				0,	JSPROP_ENUMERATE},
    {0}
};  

GET_JS_PARSE_BEGIN(Geometry_get, ChExternalObject*)
	GET_JS_PROP (0,  chjs_from_dataNEW(cx,vp, new ChCoordsys<>(this_data->GetPos()), &chjs_Coordsys)  ) 
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Geometry_set, ChExternalObject*)
	SET_JS_PROP (0,	&chjs_Coordsys, this_data->SetPos(*(Coordsys*)chjs_to_data(cx,vp))   )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsEval, ChExternalObject*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  chjs_from_dataNEW(cx, rval, new ChVector<>(this_data->Eval(chjs_to_double(cx, argv+0),
															 chjs_to_double(cx, argv+1),
															 chjs_to_double(cx, argv+2)) ), &chjs_Vector); 
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsNormal, ChExternalObject*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  chjs_from_dataNEW(cx, rval, new ChVector<>(this_data->Normal(chjs_to_double(cx, argv+0),
															   chjs_to_double(cx, argv+1),
															   chjs_to_double(cx, argv+2)) ), &chjs_Vector); 
DEF_JS_FUNEND



// ------- Method list -------------------------------
//
static JSFunctionSpec Geometry_methods[] = {
	{"Eval",			jsEval,			0},
	{"Normal",			jsNormal,		0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Geometry = {
    "Geometry", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Geometry_get,		  Geometry_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

JSClass* achjs_Geometry() {return &chjs_Geometry;} ;  // get address of static 'Javascript class' structure
  

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Geometry(JSContext* cx, JSObject* glob, JSObject* parent)
{

	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Geometry,			// this class 
				NULL, 0,				// constructor fx and parameters
				Geometry_props, Geometry_methods, 
				NULL, NULL);
	return ret;
}





} // END_OF_NAMESPACE____
