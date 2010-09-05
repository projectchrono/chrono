///////////////////////////////////////////////////
//  
//   ChJs_link.cpp
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
#include "ChJs_link.h"
#include "ChJs_funct.h"
 
#include "physics/ChSystem.h"
#include "physics/ChLinksAll.h"
#include "physics/ChGlobal.h"


namespace chrono 
{

using namespace chrono::pneumatics;


// down cast to proper js class type
/*
JSClass* chjs_cast_link(ChLink* mylink)
{
	if (!mylink) return NULL;
	switch (mylink->GetClassType())
	{ 
	case CHCLASS_LINK:
		return &chjs_Link;
	case CHCLASS_LINKSPRING:
		return &chjs_LinkSpring;
	case CHCLASS_LINKLOCK:
		return &chjs_LinkLock;
	case CHCLASS_LINKLINACTUATOR:
		return &chjs_LinkLinActuator;
	case CHCLASS_LINKPNEUMATICACTUATOR:
		return &chjs_LinkPneumaticActuator;
	case CHCLASS_LINKSCREW:
		return &chjs_LinkScrew;
	case CHCLASS_LINKGEAR:
		return &chjs_LinkGear;
	case CHCLASS_LINKENGINE:
		return &chjs_LinkEngine;
	case CHCLASS_LINKWHEEL:
		return &chjs_LinkWheel;
	case CHCLASS_LINKBRAKE:
		return &chjs_LinkBrake;
	case CHCLASS_LINKCLEARANCE:
		return &chjs_LinkClearance;

	default: 
		return &chjs_Link;
	}
	return NULL;
}
*/

JSClass* chjs_cast_link(ChLink* mylink)
{
	if (!mylink) return NULL;

	if (ChIsDerivedFromClass(ChLinkSpring,mylink))
		return &chjs_LinkSpring;
	
	if (ChIsDerivedFromClass(ChLinkLock,mylink))
	{
		if (ChIsDerivedFromClass(ChLinkLinActuator,mylink))
			return &chjs_LinkLinActuator;
		if (ChIsDerivedFromClass(ChLinkPneumaticActuator,mylink))
			return &chjs_LinkPneumaticActuator;
		if (ChIsDerivedFromClass(ChLinkScrew,mylink))
			return &chjs_LinkScrew;
		if (ChIsDerivedFromClass(ChLinkGear,mylink))
			return &chjs_LinkGear;
		if (ChIsDerivedFromClass(ChLinkEngine,mylink))
			return &chjs_LinkEngine;
		if (ChIsDerivedFromClass(ChLinkWheel,mylink))
			return &chjs_LinkWheel;
		if (ChIsDerivedFromClass(ChLinkBrake,mylink))
			return &chjs_LinkBrake;
		if (ChIsDerivedFromClass(ChLinkClearance,mylink))
			return &chjs_LinkClearance;

		return &chjs_LinkLock;
	}

	//default
	return &chjs_Link;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINK class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Link_props[] = {
    {"system",       1,	JSPROP_ENUMERATE},
    {"disabled",     4,	JSPROP_ENUMERATE},
    {"doc",		     5,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"marker1",	     6,	JSPROP_ENUMERATE},
    {"marker2",	     7,	JSPROP_ENUMERATE},
    {"relM",	     8,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"relM_dt",	     9,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"relM_dtdt",	10,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rel_angle",	11,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rel_axis",	12,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rel_Wvel",	13,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rel_Wacc",	14,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"dist",		15,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"dist_dt",		16,	JSPROP_ENUMERATE|JSPROP_READONLY},
//	{"C",			17,	JSPROP_ENUMERATE|JSPROP_READONLY},
//	{"C_dt",		18,	JSPROP_ENUMERATE|JSPROP_READONLY},
//	{"C_dtdt",		19,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"react_force",	20,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"react_torque",21,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"int_force",	22,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"int_torque",	23,	JSPROP_ENUMERATE|JSPROP_READONLY},
	{"script_force",24, JSPROP_ENUMERATE},
	{"script_torque",25,JSPROP_ENUMERATE},
	{"p",			26,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {0}
};

GET_JS_PARSE_BEGIN(Link_get, ChLink*)
	GET_JS_PROP (1,  if (this_data->GetSystem()) chjs_from_data(cx,vp,this_data->GetSystem(), &chjs_PSystem); else chjs_from_bool(cx, vp, FALSE); )
	GET_JS_PROP (4,  chjs_from_int(cx,vp,this_data->IsDisabled()) )
	GET_JS_PROP (5,  chjs_from_int(cx,vp,this_data->GetDOC()) )
//	GET_JS_PROP (6,  if (this_data->GetMarker1()) chjs_from_data(cx,vp,this_data->GetMarker1(), &chjs_Marker); else chjs_from_bool(cx, vp, FALSE); )
//	GET_JS_PROP (7,  if (this_data->GetMarker2()) chjs_from_data(cx,vp,this_data->GetMarker2(), &chjs_Marker); else chjs_from_bool(cx, vp, FALSE); )
//	GET_JS_PROP (8,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetRelM()), &chjs_Coordsys) )
//	GET_JS_PROP (9,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetRelM_dt()), &chjs_Coordsys) )
//	GET_JS_PROP (10, chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetRelM_dtdt()), &chjs_Coordsys) )
//	GET_JS_PROP (11, chjs_from_double(cx,vp, this_data->GetRelAngle()) )
//	GET_JS_PROP (12, chjs_from_dataNEW(cx,vp, new Vector(this_data->GetRelAxis()), &chjs_Vector) )
//	GET_JS_PROP (13, chjs_from_dataNEW(cx,vp, new Vector(this_data->GetRelWvel()), &chjs_Vector) )
//	GET_JS_PROP (14, chjs_from_dataNEW(cx,vp, new Vector(this_data->GetRelWacc()), &chjs_Vector) )
//	GET_JS_PROP (15, chjs_from_double(cx,vp, this_data->GetDist()) )
//	GET_JS_PROP (16, chjs_from_double(cx,vp, this_data->GetDist_dt()) )
	//GET_JS_PROP (17, chjs_from_data(cx,vp,this_data->GetC(), &chjs_Matrix) )
	//GET_JS_PROP (18, chjs_from_data(cx,vp,this_data->GetC_dt(), &chjs_Matrix) )
	//GET_JS_PROP (19, chjs_from_data(cx,vp,this_data->GetC_dtdt(), &chjs_Matrix) )
	GET_JS_PROP (20, chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_react_force()), &chjs_Vector) )
	GET_JS_PROP (21, chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_react_torque()), &chjs_Vector) )
//	GET_JS_PROP (22, chjs_from_dataNEW(cx,vp, new Vector(this_data->GetC_force()), &chjs_Vector) )
//	GET_JS_PROP (23, chjs_from_dataNEW(cx,vp, new Vector(this_data->GetC_torque()), &chjs_Vector) )
//	GET_JS_PROP (24, chjs_from_data(cx,vp, this_data->Get_Scr_force(), &chjs_Vector) )
//	GET_JS_PROP (25, chjs_from_data(cx,vp, this_data->Get_Scr_torque(), &chjs_Vector) )
//	GET_JS_PROP (26, chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetMarker2()->GetAbsCoord()), &chjs_Coordsys)  )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Link_set, ChLink*)
	SET_JS_PROP (1,	&chjs_PSystem, this_data->SetSystem((ChSystem*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (4,	&chjs_boolean, this_data->SetDisabled(chjs_to_bool(cx,vp)) )
//	SET_JS_PROP (6,	&chjs_Marker, this_data->SetMarker1((ChMarker*)chjs_to_data(cx,vp)) )
//	SET_JS_PROP (7,	&chjs_Marker, this_data->SetMarker2((ChMarker*)chjs_to_data(cx,vp)) )
//	SET_JS_PROP (24,&chjs_Vector, this_data->Set_Scr_force(*(Vector*)chjs_to_data(cx,vp)) )
//	SET_JS_PROP (25,&chjs_Vector, this_data->Set_Scr_torque(*(Vector*)chjs_to_data(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsUpdate, ChLink*, 0)
  this_data->Update();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND




// ------- Method list -------------------------------
//
static JSFunctionSpec Link_methods[] = {
	{"Update",			jsUpdate,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Link = {
    "Link", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Link_get,		  Link_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Link(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Link,				// this class 
				NULL, 0,				// constructor fx and parameters
				Link_props, Link_methods, 
				NULL, NULL);
	return ret;
}







//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKLOCK class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkLock_props[] = {
    {"delta",	     1,	JSPROP_ENUMERATE},
    {"delta_dt",	 2,	JSPROP_ENUMERATE},
    {"delta_dtdt",	 3,	JSPROP_ENUMERATE},
	{"motion_x",	 4,	JSPROP_ENUMERATE},
	{"motion_y",	 5,	JSPROP_ENUMERATE},
	{"motion_z",	 6,	JSPROP_ENUMERATE},
	{"motion_a1",	 7,	JSPROP_ENUMERATE},
	{"motion_a2",	 8,	JSPROP_ENUMERATE},
	{"motion_a2",	 9,	JSPROP_ENUMERATE},
	{"motion_axis",	 10,	JSPROP_ENUMERATE},
	{"motion_angleset",11,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(LinkLock_get, ChLinkLock*)
	GET_JS_PROP (1,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetDeltaC()), &chjs_Coordsys) )
	GET_JS_PROP (2,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetDeltaC_dt()), &chjs_Coordsys) )
	GET_JS_PROP (3,  chjs_from_dataNEW(cx,vp, new Coordsys(this_data->GetDeltaC_dtdt()), &chjs_Coordsys) )
	GET_JS_PROP (4,  chjs_from_data(cx,vp,this_data->GetMotion_X(), chjs_cast_funct(this_data->GetMotion_X() )) )
	GET_JS_PROP (5,  chjs_from_data(cx,vp,this_data->GetMotion_Y(), chjs_cast_funct(this_data->GetMotion_Y() )) )
	GET_JS_PROP (6,  chjs_from_data(cx,vp,this_data->GetMotion_Z(), chjs_cast_funct(this_data->GetMotion_Z() )) )
	GET_JS_PROP (7,  chjs_from_data(cx,vp,this_data->GetMotion_ang(), chjs_cast_funct(this_data->GetMotion_ang() )) )
	GET_JS_PROP (8,  chjs_from_data(cx,vp,this_data->GetMotion_ang2(), chjs_cast_funct(this_data->GetMotion_ang2() )) )
	GET_JS_PROP (9,  chjs_from_data(cx,vp,this_data->GetMotion_ang3(), chjs_cast_funct(this_data->GetMotion_ang3() )) )
	GET_JS_PROP (10, chjs_from_dataNEW(cx,vp, new Vector(this_data->GetMotion_axis()), &chjs_Vector) )
	GET_JS_PROP (11, chjs_from_int(cx,vp, this_data->Get_angleset()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkLock_set, ChLinkLock*)
	SET_JS_PROP (1,	&chjs_Coordsys, this_data->SetDeltaC(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (2,	&chjs_Coordsys, this_data->SetDeltaC_dt(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (3,	&chjs_Coordsys, this_data->SetDeltaC_dtdt(*(Coordsys*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (4,	&chjs_Function, this_data->SetMotion_X((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (5,	&chjs_Function, this_data->SetMotion_Y((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (6,	&chjs_Function, this_data->SetMotion_Z((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (7,	&chjs_Function, this_data->SetMotion_ang ((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (8,	&chjs_Function, this_data->SetMotion_ang2((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (9,	&chjs_Function, this_data->SetMotion_ang3((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (10,&chjs_Vector,   this_data->SetMotion_axis(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (11,&chjs_int,      this_data->Set_angleset(chjs_to_int(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkLock_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkLock = {
    "LinkLock", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkLock_get,		  LinkLock_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkLock(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkLock,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkLock_props, LinkLock_methods, 
				NULL, NULL);
	return ret;
}





//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKSPRING class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkSpring_props[] = {
    {"k",	     1,	JSPROP_ENUMERATE},
    {"r",		 2,	JSPROP_ENUMERATE},
    {"f",		 3,	JSPROP_ENUMERATE},
    {"d_rest",	 4,	JSPROP_ENUMERATE},
	{"mod_f_time",  5, JSPROP_ENUMERATE},
	{"mod_k_d",	    6, JSPROP_ENUMERATE},
	{"mod_r_d",	    7, JSPROP_ENUMERATE},
	{"mod_k_speed", 8, JSPROP_ENUMERATE},
	{"mod_r_speed", 9, JSPROP_ENUMERATE},
	{"spring_react",10, JSPROP_ENUMERATE | JSPROP_READONLY},
    {0}
};

GET_JS_PARSE_BEGIN(LinkSpring_get, ChLinkSpring*)
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->Get_SpringK()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_SpringR()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp, this_data->Get_SpringF()) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp, this_data->Get_SpringRestLenght()) )
	GET_JS_PROP (5,  chjs_from_data(cx,vp,this_data->Get_mod_f_time(), chjs_cast_funct(this_data->Get_mod_f_time() )) )
	GET_JS_PROP (6,  chjs_from_data(cx,vp,this_data->Get_mod_k_d(), chjs_cast_funct(this_data->Get_mod_k_d() )) )
	GET_JS_PROP (7,  chjs_from_data(cx,vp,this_data->Get_mod_r_d(), chjs_cast_funct(this_data->Get_mod_r_d() )) )
	GET_JS_PROP (8,  chjs_from_data(cx,vp,this_data->Get_mod_k_speed(), chjs_cast_funct(this_data->Get_mod_k_speed() )) )
	GET_JS_PROP (9,  chjs_from_data(cx,vp,this_data->Get_mod_r_speed(), chjs_cast_funct(this_data->Get_mod_r_speed() )) )
	GET_JS_PROP (10, chjs_from_double(cx,vp,this_data->Get_SpringReact()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkSpring_set, ChLinkSpring*)
	SET_JS_PROP (1,  &chjs_double,  this_data->Set_SpringK(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,  this_data->Set_SpringR(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,  &chjs_double,  this_data->Set_SpringF(chjs_to_double(cx,vp)) )
	SET_JS_PROP (4,  &chjs_double,  this_data->Set_SpringRestLenght(chjs_to_double(cx,vp)) )
	SET_JS_PROP (5,	 &chjs_Function,this_data->Set_mod_f_time((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (6,	 &chjs_Function,this_data->Set_mod_k_d((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (7,	 &chjs_Function,this_data->Set_mod_r_d((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (8,	 &chjs_Function,this_data->Set_mod_k_speed((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (9,	 &chjs_Function,this_data->Set_mod_r_speed((ChFunction*)chjs_to_data(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkSpring_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkSpring = {
    "LinkSpring", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkSpring_get,		  LinkSpring_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkSpring(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkSpring,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkSpring_props, LinkSpring_methods, 
				NULL, NULL);
	return ret;
}





//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKLinActuator class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkLinActuator_props[] = {
    {"dist_funct",	1,	JSPROP_ENUMERATE},
    {"offset",		2,	JSPROP_ENUMERATE},
    {"learn",		3,	JSPROP_ENUMERATE},
    {"tau",			4,	JSPROP_ENUMERATE},
    {"eta",			5,	JSPROP_ENUMERATE},
    {"inertia",		6,	JSPROP_ENUMERATE},
    {"rerot",		7,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"rerot_dt",	8,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"rerot_dtdt",	9,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"retorque",  10,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {0}
};

GET_JS_PARSE_BEGIN(LinkLinActuator_get, ChLinkLinActuator*)
	GET_JS_PROP (1,  chjs_from_data(cx,vp,this_data->Get_dist_funct(), chjs_cast_funct(this_data->Get_dist_funct() )) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_lin_offset()) )
	GET_JS_PROP (3,  chjs_from_int(cx,vp, this_data->Get_learn()) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp, this_data->Get_mot_tau()) )
	GET_JS_PROP (5,  chjs_from_double(cx,vp, this_data->Get_mot_eta()) )
	GET_JS_PROP (6,  chjs_from_double(cx,vp, this_data->Get_mot_inertia()) )
	GET_JS_PROP (7,  chjs_from_double(cx,vp, this_data->Get_mot_rerot()) )
	GET_JS_PROP (8,  chjs_from_double(cx,vp, this_data->Get_mot_rerot_dt()) )
	GET_JS_PROP (9,  chjs_from_double(cx,vp, this_data->Get_mot_rerot_dtdt()) )
	GET_JS_PROP (10, chjs_from_double(cx,vp, this_data->Get_mot_retorque()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkLinActuator_set, ChLinkLinActuator*)
	SET_JS_PROP (1,	 &chjs_Function, this_data->Set_dist_funct((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,   this_data->Set_lin_offset(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,  &chjs_int,	 	 this_data->Set_learn(chjs_to_int(cx,vp)) )
	SET_JS_PROP (4,  &chjs_double,   this_data->Set_mot_tau(chjs_to_double(cx,vp)) )
	SET_JS_PROP (5,  &chjs_double,   this_data->Set_mot_eta(chjs_to_double(cx,vp)) )
	SET_JS_PROP (6,  &chjs_double,   this_data->Set_mot_inertia(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkLinActuator_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkLinActuator = {
    "LinkLinActuator", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkLinActuator_get,		  LinkLinActuator_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkLinActuator(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkLinActuator,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkLinActuator_props, LinkLinActuator_methods, 
				NULL, NULL);
	return ret;
}




//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKPneumaticActuator class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkPneumaticActuator_props[] = {
    {"offset",	0,	JSPROP_ENUMERATE},
    {"comA",	1,	JSPROP_ENUMERATE},
    {"comB",	2,	JSPROP_ENUMERATE},
    {"pA",		3,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"pB",		4,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"pA_dt",	5,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"pB_dt",	6,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"pneu_F",  7,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"pneu_pos",8,	JSPROP_ENUMERATE | JSPROP_READONLY},
    {"pneu_pos_dt",9,JSPROP_ENUMERATE | JSPROP_READONLY},
    {"Ci",		10,	JSPROP_ENUMERATE},
    {"Co",		11,	JSPROP_ENUMERATE},
    {"Bi",		12,	JSPROP_ENUMERATE},
    {"Bo",		13,	JSPROP_ENUMERATE},
    {"Ps",		14,	JSPROP_ENUMERATE},
    {"Pma",		15,	JSPROP_ENUMERATE},
    {"Pmb",		16,	JSPROP_ENUMERATE},
    {"L",		17,	JSPROP_ENUMERATE},
    {"Wa",		18,	JSPROP_ENUMERATE},
    {"Wb",		19,	JSPROP_ENUMERATE},
    {"area",	20,	JSPROP_ENUMERATE},
    {"alfa",	21,	JSPROP_ENUMERATE},
    {"gamma",	22,	JSPROP_ENUMERATE},
    {"vA_min",	23,	JSPROP_ENUMERATE},
    {"vA_max",	24,	JSPROP_ENUMERATE},
    {"vA_close",25,	JSPROP_ENUMERATE},
    {"vB_min",	26,	JSPROP_ENUMERATE},
    {"vB_max",	27,	JSPROP_ENUMERATE},
    {"vB_close",28,	JSPROP_ENUMERATE},

    {0}
};

GET_JS_PARSE_BEGIN(LinkPneumaticActuator_get, ChLinkPneumaticActuator*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp, this_data->Get_lin_offset()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->Get_ComA() ) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_ComB() ) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp, this_data->Get_pA() ) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp, this_data->Get_pB() ) )
	GET_JS_PROP (5,  chjs_from_double(cx,vp, this_data->Get_pA_dt() ) )
	GET_JS_PROP (6,  chjs_from_double(cx,vp, this_data->Get_pB_dt() ) )
	GET_JS_PROP (7,  chjs_from_double(cx,vp, this_data->Get_pneu_F() ) )
	GET_JS_PROP (8,  chjs_from_double(cx,vp, this_data->Get_pneu_pos() ) )
	GET_JS_PROP (9,  chjs_from_double(cx,vp, this_data->Get_pneu_pos_dt() ) )
	GET_JS_PROP (10, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Ci() ) )
	GET_JS_PROP (11, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Co() ) )
	GET_JS_PROP (12, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Bi() ) )
	GET_JS_PROP (13, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Bo() ) )
	GET_JS_PROP (14, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Ps() ) )
	GET_JS_PROP (15, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Pma() ) )
	GET_JS_PROP (16, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Pmb() ) )
	GET_JS_PROP (17, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_L() ) )
	GET_JS_PROP (18, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Wa() ) )
	GET_JS_PROP (19, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Wb() ) )
	GET_JS_PROP (20, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_A() ) )
	GET_JS_PROP (21, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Alfa() ) )
	GET_JS_PROP (22, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_Gamma() ) )
	GET_JS_PROP (23, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_ValvA_min() ) )
	GET_JS_PROP (24, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_ValvA_max() ) )
	GET_JS_PROP (25, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_ValvA_close() ))
	GET_JS_PROP (26, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_ValvB_min() ) )
	GET_JS_PROP (27, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_ValvB_max() ) )
	GET_JS_PROP (28, chjs_from_double(cx,vp, this_data->Get_pneuma()->Get_ValvB_close() ))
GET_JS_PARSE_END 


SET_JS_PARSE_BEGIN(LinkPneumaticActuator_set, ChLinkPneumaticActuator*)
	SET_JS_PROP (0,  &chjs_double,   this_data->Set_lin_offset(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,  &chjs_double,   this_data->Set_ComA(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,   this_data->Set_ComB(chjs_to_double(cx,vp)) )
	SET_JS_PROP (10, &chjs_double,   this_data->Get_pneuma()->Set_Ci(chjs_to_double(cx,vp)) )
	SET_JS_PROP (11, &chjs_double,   this_data->Get_pneuma()->Set_Co(chjs_to_double(cx,vp)) )
	SET_JS_PROP (12, &chjs_double,   this_data->Get_pneuma()->Set_Bi(chjs_to_double(cx,vp)) )
	SET_JS_PROP (13, &chjs_double,   this_data->Get_pneuma()->Set_Bo(chjs_to_double(cx,vp)) )
	SET_JS_PROP (14, &chjs_double,   this_data->Get_pneuma()->Set_Ps(chjs_to_double(cx,vp)) )
	SET_JS_PROP (15, &chjs_double,   this_data->Get_pneuma()->Set_Pma(chjs_to_double(cx,vp)) )
	SET_JS_PROP (16, &chjs_double,   this_data->Get_pneuma()->Set_Pmb(chjs_to_double(cx,vp)) )
	SET_JS_PROP (17, &chjs_double,   this_data->Get_pneuma()->Set_L(chjs_to_double(cx,vp)) )
	SET_JS_PROP (18, &chjs_double,   this_data->Get_pneuma()->Set_Wa(chjs_to_double(cx,vp)) )
	SET_JS_PROP (19, &chjs_double,   this_data->Get_pneuma()->Set_Wb(chjs_to_double(cx,vp)) )
	SET_JS_PROP (20, &chjs_double,   this_data->Get_pneuma()->Set_A(chjs_to_double(cx,vp)) )
	SET_JS_PROP (21, &chjs_double,   this_data->Get_pneuma()->Set_Alfa(chjs_to_double(cx,vp)) )
	SET_JS_PROP (22, &chjs_double,   this_data->Get_pneuma()->Set_Gamma(chjs_to_double(cx,vp)) )
	SET_JS_PROP (23, &chjs_double,   this_data->Get_pneuma()->Set_ValvA_min(chjs_to_double(cx,vp)) )
	SET_JS_PROP (24, &chjs_double,   this_data->Get_pneuma()->Set_ValvA_max(chjs_to_double(cx,vp)) )
	SET_JS_PROP (25, &chjs_double,   this_data->Get_pneuma()->Set_ValvA_close(chjs_to_double(cx,vp)) )
	SET_JS_PROP (26, &chjs_double,   this_data->Get_pneuma()->Set_ValvB_min(chjs_to_double(cx,vp)) )
	SET_JS_PROP (27, &chjs_double,   this_data->Get_pneuma()->Set_ValvB_max(chjs_to_double(cx,vp)) )
	SET_JS_PROP (28, &chjs_double,   this_data->Get_pneuma()->Set_ValvB_close(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsSetupActuator, ChLinkPneumaticActuator*, 0)
  this_data->SetupActuator();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND



// ------- Method list -------------------------------
//
static JSFunctionSpec LinkPneumaticActuator_methods[] = {
	{"SetupActuator",			jsSetupActuator,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkPneumaticActuator = {
    "LinkPneumaticActuator", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkPneumaticActuator_get,	 LinkPneumaticActuator_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkPneumaticActuator(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkPneumaticActuator,// this class 
				NULL, 0,				// constructor fx and parameters
				LinkPneumaticActuator_props, LinkPneumaticActuator_methods, 
				NULL, NULL);
	return ret;
}






//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKScrew class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkScrew_props[] = {
    {"tau",	     1,	JSPROP_ENUMERATE},
    {"thread",	 2,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(LinkScrew_get, ChLinkScrew*)
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->Get_tau()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_thread()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkScrew_set, ChLinkScrew*)
	SET_JS_PROP (1,  &chjs_double,   this_data->Set_tau(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,   this_data->Set_thread(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkScrew_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkScrew = {
    "LinkScrew", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkScrew_get,		  LinkScrew_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkScrew(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkScrew,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkScrew_props, LinkScrew_methods, 
				NULL, NULL);
	return ret;
}





//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKGear class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkGear_props[] = {
    {"tau",	     1,	JSPROP_ENUMERATE},
    {"alpha",	 2,	JSPROP_ENUMERATE},
    {"beta",	 3,	JSPROP_ENUMERATE},
    {"inner",	 4,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(LinkGear_get, ChLinkGear*)
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->Get_tau()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_alpha()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp, this_data->Get_beta()) )
	GET_JS_PROP (4,  chjs_from_int(cx,vp, this_data->Get_epicyclic()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkGear_set, ChLinkGear*)
	SET_JS_PROP (1,  &chjs_double,   this_data->Set_tau(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,   this_data->Set_alpha(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,  &chjs_double,   this_data->Set_beta(chjs_to_double(cx,vp)) )
	SET_JS_PROP (4,  &chjs_int,      this_data->Set_epicyclic(chjs_to_int(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//


// ------- Method list -------------------------------
//
static JSFunctionSpec LinkGear_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkGear = {
    "LinkGear", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkGear_get,		  LinkGear_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkGear(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkGear,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkGear_props, LinkGear_methods, 
				NULL, NULL);
	return ret;
}






//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKEngine class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkEngine_props[] = {
    {"rot_funct",	     1,	JSPROP_ENUMERATE},
    {"spe_funct",	     2,	JSPROP_ENUMERATE},
    {"tor_funct",	     3,	JSPROP_ENUMERATE},
    {"torquew_funct",	 4,	JSPROP_ENUMERATE},
    {"learn",			 5,	JSPROP_ENUMERATE},
    {"apply_reducer",	 6,	JSPROP_ENUMERATE},
    {"eng_mode",		 7,	JSPROP_ENUMERATE},
    {"shaft_mode",		 8,	JSPROP_ENUMERATE},
    {"tau",				 9,	JSPROP_ENUMERATE},
    {"eta",				10, JSPROP_ENUMERATE},
    {"inertia",			11, JSPROP_ENUMERATE},
    {"rot",				12, JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rot_dt",			13, JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rot_dtdt",		14, JSPROP_ENUMERATE|JSPROP_READONLY},
    {"torque",			15, JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rerot",			16, JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rerot_dt",		17, JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rerot_dtdt",		18, JSPROP_ENUMERATE|JSPROP_READONLY},
    {"retorque",		19, JSPROP_ENUMERATE|JSPROP_READONLY},
    {0}
};

GET_JS_PARSE_BEGIN(LinkEngine_get, ChLinkEngine*)
	GET_JS_PROP (1,  chjs_from_data(cx,vp,this_data->Get_rot_funct(), chjs_cast_funct(this_data->Get_rot_funct() )) )
	GET_JS_PROP (2,  chjs_from_data(cx,vp,this_data->Get_spe_funct(), chjs_cast_funct(this_data->Get_spe_funct() )) )
	GET_JS_PROP (3,  chjs_from_data(cx,vp,this_data->Get_tor_funct(), chjs_cast_funct(this_data->Get_tor_funct() )) )
	GET_JS_PROP (4,  chjs_from_data(cx,vp,this_data->Get_torque_w_funct(), chjs_cast_funct(this_data->Get_torque_w_funct() )) )
	GET_JS_PROP (5,  chjs_from_int(cx,vp, this_data->Get_learn()) )
	GET_JS_PROP (6,  chjs_from_int(cx,vp, this_data->Get_impose_reducer()) )
	GET_JS_PROP (7,  chjs_from_int(cx,vp, this_data->Get_eng_mode()) )
	GET_JS_PROP (8,  chjs_from_int(cx,vp, this_data->Get_shaft_mode()) )
	GET_JS_PROP (9,  chjs_from_double(cx,vp, this_data->Get_mot_eta()) )
	GET_JS_PROP (10, chjs_from_double(cx,vp, this_data->Get_mot_tau()) )
	GET_JS_PROP (11, chjs_from_double(cx,vp, this_data->Get_mot_inertia()) )
	GET_JS_PROP (12, chjs_from_double(cx,vp, this_data->Get_mot_rot()) )
	GET_JS_PROP (13, chjs_from_double(cx,vp, this_data->Get_mot_rot_dt()) )
	GET_JS_PROP (14, chjs_from_double(cx,vp, this_data->Get_mot_rot_dtdt()) )
	GET_JS_PROP (15, chjs_from_double(cx,vp, this_data->Get_mot_torque()) )
	GET_JS_PROP (16, chjs_from_double(cx,vp, this_data->Get_mot_rerot()) )
	GET_JS_PROP (17, chjs_from_double(cx,vp, this_data->Get_mot_rerot_dt()) )
	GET_JS_PROP (18, chjs_from_double(cx,vp, this_data->Get_mot_rerot_dtdt()) )
	GET_JS_PROP (19, chjs_from_double(cx,vp, this_data->Get_mot_retorque()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkEngine_set, ChLinkEngine*)
	SET_JS_PROP (1,	 &chjs_Function, this_data->Set_rot_funct((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (2,	 &chjs_Function, this_data->Set_spe_funct((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (3,	 &chjs_Function, this_data->Set_tor_funct((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (4,	 &chjs_Function, this_data->Set_torque_w_funct((ChFunction*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (5,  &chjs_int,	 	 this_data->Set_learn(chjs_to_int(cx,vp)) )
	SET_JS_PROP (6,  &chjs_int,	 	 this_data->Set_impose_reducer(chjs_to_int(cx,vp)) )
	SET_JS_PROP (7,  &chjs_int,	 	 this_data->Set_eng_mode(chjs_to_int(cx,vp)) )
	SET_JS_PROP (8,  &chjs_int,	 	 this_data->Set_shaft_mode(chjs_to_int(cx,vp)) )
	SET_JS_PROP (9,  &chjs_double,	 this_data->Set_mot_eta(chjs_to_double(cx,vp)) )
	SET_JS_PROP (10, &chjs_double,	 this_data->Set_mot_tau(chjs_to_double(cx,vp)) )
	SET_JS_PROP (11, &chjs_double,	 this_data->Set_mot_inertia(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkEngine_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkEngine = {
    "LinkEngine", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkEngine_get,		  LinkEngine_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkEngine(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkEngine,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkEngine_props, LinkEngine_methods, 
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKbrake class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkBrake_props[] = {
    {"braking",	     1,	JSPROP_ENUMERATE},
    {"stick_ratio",	 2,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(LinkBrake_get, ChLinkBrake*)
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->Get_brake_torque()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_stick_ratio()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkBrake_set, ChLinkBrake*)
	SET_JS_PROP (1,  &chjs_double,	 this_data->Set_brake_torque(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,	 this_data->Set_stick_ratio(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkBrake_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkBrake = {
    "LinkBrake", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkBrake_get,	  LinkBrake_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkBrake(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkBrake,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkBrake_props, LinkBrake_methods, 
				NULL, NULL);
	return ret;
}





//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKclearance class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkClearance_props[] = {
    {"clearance",    1,	JSPROP_ENUMERATE},
    {"c_friction",	 2,	JSPROP_ENUMERATE},
    {"c_restitution",3,	JSPROP_ENUMERATE},
    {"c_tang_restitution", 4,	JSPROP_ENUMERATE},
    {"diameter",	 5,	JSPROP_ENUMERATE},
    {"axis_eccentricity",	 6,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"axis_phase",			 7,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"rotation_angle",		 8,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"contact_P_abs",		 9,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"contact_N_abs",		10,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"contact_F_abs",		11,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"contact_F_n",			12,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"contact_F_t",			13,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"contact_V_t",			14,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {0}
};
  
GET_JS_PARSE_BEGIN(LinkClearance_get, ChLinkClearance*)
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->Get_clearance()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_c_friction()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp, this_data->Get_c_restitution()) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp, this_data->Get_c_tang_restitution()) )
	GET_JS_PROP (5,  chjs_from_double(cx,vp, this_data->Get_diameter()) )
	GET_JS_PROP (6,  chjs_from_double(cx,vp, this_data->Get_axis_eccentricity()) )
	GET_JS_PROP (7,  chjs_from_double(cx,vp, this_data->Get_axis_phase()) )
	GET_JS_PROP (8,  chjs_from_double(cx,vp, this_data->Get_rotation_angle()) )
	GET_JS_PROP (9,  chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_contact_P_abs()), &chjs_Vector) )
	GET_JS_PROP (10, chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_contact_N_abs()), &chjs_Vector) )
	GET_JS_PROP (11, chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_contact_F_abs()), &chjs_Vector) )
	GET_JS_PROP (12, chjs_from_double(cx,vp, this_data->Get_contact_F_n()) )
	GET_JS_PROP (13, chjs_from_double(cx,vp, this_data->Get_contact_F_t()) )
	GET_JS_PROP (14, chjs_from_double(cx,vp, this_data->Get_contact_V_t()) )
GET_JS_PARSE_END

 
SET_JS_PARSE_BEGIN(LinkClearance_set, ChLinkClearance*)
	SET_JS_PROP (1,  &chjs_double,	 this_data->Set_clearance(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,	 this_data->Set_c_friction(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,  &chjs_double,	 this_data->Set_c_restitution(chjs_to_double(cx,vp)) )
	SET_JS_PROP (4,  &chjs_double,	 this_data->Set_c_tang_restitution(chjs_to_double(cx,vp)) )
	SET_JS_PROP (5,  &chjs_double,	 this_data->Set_diameter(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkClearance_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkClearance = {
    "LinkClearance", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkClearance_get,	  LinkClearance_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkClearance(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkClearance,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkClearance_props, LinkClearance_methods, 
				NULL, NULL);
	return ret;
}





//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   LINKwheel class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec LinkWheel_props[] = {
    {"radius",	     1,	JSPROP_ENUMERATE},
    {"friction",	 2,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(LinkWheel_get, ChLinkWheel*)
	GET_JS_PROP (1,  chjs_from_double(cx,vp, this_data->Get_radius()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp, this_data->Get_friction()) )
GET_JS_PARSE_END


SET_JS_PARSE_BEGIN(LinkWheel_set, ChLinkWheel*)
	SET_JS_PROP (1,  &chjs_double,	 this_data->Set_radius(chjs_to_int(cx,vp)) )
	SET_JS_PROP (2,  &chjs_double,	 this_data->Set_friction(chjs_to_int(cx,vp)) )
SET_JS_PARSE_END


  
////////////////////////////////////////////////////////////////////
//
// METHODS
//




// ------- Method list -------------------------------
//
static JSFunctionSpec LinkWheel_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_LinkWheel = {
    "LinkWheel", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  LinkWheel_get,		  LinkWheel_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_LinkWheel(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_LinkWheel,				// this class 
				NULL, 0,				// constructor fx and parameters
				LinkWheel_props, LinkWheel_methods, 
				NULL, NULL);
	return ret;
}


} // END_OF_NAMESPACE____

