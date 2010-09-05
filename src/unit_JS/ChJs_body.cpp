///////////////////////////////////////////////////
//  
//   ChJs_body.cpp
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
#include "ChJs_marker.h"
#include "ChJs_force.h"
#include "ChJs_geometry.h"

#include "physics/ChGlobal.h"
#include "physics/ChSystem.h"
#include "physics/ChBody.h"
#include "physics/ChExternalObject.h"




namespace chrono 
{


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   BODY class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec RBody_props[] = {
    {"mass",       0,	JSPROP_ENUMERATE},
    {"inertia",    1,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"system",     2,	JSPROP_ENUMERATE},
    {"p",		   3,	JSPROP_ENUMERATE},
    {"p_dt",       4,	JSPROP_ENUMERATE},
    {"p_dtdt",     5,	JSPROP_ENUMERATE},
    {"Wvel",	   6,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Wacc",	   7,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Wvel_abs",   8,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Wacc_abs",   9,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rot_axis",   10,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rot_angle",  11,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"A",		   12,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"lock",	   16,	JSPROP_ENUMERATE},
    {"collide",	   17,	JSPROP_ENUMERATE},
    {"impactC",	   18,	JSPROP_ENUMERATE},
    {"impactCt",   19,	JSPROP_ENUMERATE},
	{"Sfriction",  20,	JSPROP_ENUMERATE},
	{"Kfriction",  21,	JSPROP_ENUMERATE},
    {"script_force",22,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"script_torque",23,JSPROP_ENUMERATE|JSPROP_READONLY},
    {0}
};  

GET_JS_PARSE_BEGIN(RBody_get, ChBody*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->GetMass()) )
	GET_JS_PROP (1,  chjs_from_data(cx,vp,this_data->GetXInertia(), &chjs_Matrix) )
	GET_JS_PROP (2,  chjs_from_data(cx,vp,this_data->GetSystem(), &chjs_PSystem) )
	GET_JS_PROP (3,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetCoord()), &chjs_Coordsys)  )
	GET_JS_PROP (4,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetCoord_dt()), &chjs_Coordsys)  )
	GET_JS_PROP (5,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetCoord_dtdt()), &chjs_Coordsys)  )
	GET_JS_PROP (6,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetWvel_loc()), &chjs_Vector)  )
	GET_JS_PROP (7,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetWacc_loc()), &chjs_Vector)  )
	GET_JS_PROP (8,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetWvel_par()), &chjs_Vector)  )
	GET_JS_PROP (9,  chjs_from_dataNEW(cx,vp, new Vector(this_data->GetWacc_par()), &chjs_Vector)  )
	GET_JS_PROP (10, chjs_from_dataNEW(cx,vp, new Vector(this_data->GetRotAxis()), &chjs_Vector)  )
	GET_JS_PROP (11, chjs_from_double(cx,vp,this_data->GetRotAngle()) )
	GET_JS_PROP (12, chjs_from_data(cx,vp,this_data->GetA(), &chjs_Matrix) )
	GET_JS_PROP (16, chjs_from_int(cx,vp,this_data->GetBodyFixed()) )
	GET_JS_PROP (17, chjs_from_int(cx,vp,this_data->GetCollide()) )
	GET_JS_PROP (18, chjs_from_double(cx,vp,this_data->GetImpactC()) )
	GET_JS_PROP (19, chjs_from_double(cx,vp,this_data->GetImpactCt()) )
	GET_JS_PROP (20, chjs_from_double(cx,vp,this_data->GetSfriction()) )
	GET_JS_PROP (21, chjs_from_double(cx,vp,this_data->GetKfriction()) )
	GET_JS_PROP (22, chjs_from_data(cx,vp, this_data->Get_Scr_force(), &chjs_Vector) )
	GET_JS_PROP (23, chjs_from_data(cx,vp, this_data->Get_Scr_torque(), &chjs_Vector) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(RBody_set, ChBody*)
	SET_JS_PROP (0,	&chjs_double, this_data->SetMass(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_Matrix, this_data->SetInertia((ChMatrix33<>*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (2,	&chjs_PSystem, this_data->SetSystem((ChSystem*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (3,	&chjs_Coordsys, this_data->SetCoord(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (4,	&chjs_Coordsys, this_data->SetCoord_dt(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (5,	&chjs_Coordsys, this_data->SetCoord_dtdt(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (16,&chjs_int, this_data->SetBodyFixed(chjs_to_int(cx,vp)) )
	SET_JS_PROP (17,&chjs_int, this_data->SetCollide(chjs_to_int(cx,vp)) )
	SET_JS_PROP (18,&chjs_double, this_data->SetImpactC(chjs_to_double(cx,vp)) )
	SET_JS_PROP (19,&chjs_double, this_data->SetImpactCt(chjs_to_double(cx,vp)) )
	SET_JS_PROP (20,&chjs_double, this_data->SetSfriction(chjs_to_double(cx,vp)) )
	SET_JS_PROP (21,&chjs_double, this_data->SetKfriction(chjs_to_double(cx,vp)) )
	SET_JS_PROP (22,&chjs_Vector, this_data->Set_Scr_force(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (23,&chjs_Vector, this_data->Set_Scr_torque(*(Vector*)chjs_to_data(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsUpdate, ChBody*, 0)
  this_data->Update();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMove,  ChBody*, 1)
  PARCHK(0, &chjs_Coordsys);
  this_data->Move(*(Coordsys*)chjs_to_data(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsObj, ChBody*, 1)
  PARCHK(0, &chjs_string);
  ChMarker* mma = this_data->SearchMarker(chjs_to_string(cx, argv+0));
  if (mma) { chjs_from_data(cx,rval, mma, &chjs_Marker); return JS_TRUE; }
  ChForce* mfa = this_data->SearchForce(chjs_to_string(cx, argv+0));
  if (mfa) { chjs_from_data(cx,rval, mfa, &chjs_Force); return JS_TRUE; }
  JS_ReportError(cx,"No CHRONO object '%s' in body", chjs_to_string(cx, argv+0)); 
  return JS_FALSE;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsGeometry, ChBody*, 1)
  PARCHK(0, &chjs_string);
  ChExternalObject* mext = this_data->GetExternalObject()->GetChildByName(chjs_to_string(cx, argv+0));
  if (mext) { chjs_from_dataNEW(cx,rval, mext, &chjs_Geometry); return JS_TRUE; }
  JS_ReportError(cx,"No 3d geometric object '%s' in body", chjs_to_string(cx, argv+0)); 
  return JS_FALSE; 
 //***OLD VERSION***
 // R3OBJ* mobj = NULL;
 // R3OBJ* inobj = ((ChExternalObjectR3D*)this_data->GetExternalObject())->Get_r3d_object();
 // mobj = ChFindByName(inobj, chjs_to_string(cx, argv+0));
 // if (mobj) { chjs_from_data(cx,rval, mobj, &chjs_Geometry); return JS_TRUE; }
 // JS_ReportError(cx,"No 3d geometric object '%s' in body", chjs_to_string(cx, argv+0)); 
 // return JS_FALSE;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsAccumulate_script_force,  ChBody*, 3)
  PARCHK(0, &chjs_Vector);
  PARCHK(1, &chjs_Vector);
  PARCHK(2, &chjs_int);
  this_data->Accumulate_script_force( *(Vector*)chjs_to_data(cx, argv+0) ,
									  *(Vector*)chjs_to_data(cx, argv+1) ,
									  chjs_to_int(cx, argv+2)  );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsAccumulate_script_torque,  ChBody*, 2)
  PARCHK(0, &chjs_Vector);
  PARCHK(1, &chjs_int);
  this_data->Accumulate_script_torque( *(Vector*)chjs_to_data(cx, argv+0) ,
									   chjs_to_int(cx, argv+1)  );
DEF_JS_FUNEND

// ------- Method list -------------------------------
//
static JSFunctionSpec RBody_methods[] = {
	{"Update",			jsUpdate,			0},
	{"Move",			jsMove,				0},
	{"Obj",				jsObj,				0},
	{"Geo",				jsGeometry,			0},
	{"Accumulate_script_force",	 jsAccumulate_script_force,			0},
	{"Accumulate_script_torque", jsAccumulate_script_torque,		0},
    {0}
};
 

////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_RBody = {
    "Body", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  RBody_get,		  RBody_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

JSClass* achjs_RBody() {return &chjs_RBody;}

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_RBody(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_RBody,			// this class 
				NULL, 0,				// constructor fx and parameters
				RBody_props, RBody_methods, 
				NULL, NULL);
	return ret;
}



} // END_OF_NAMESPACE____
