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
#include "ChJs_math.h"
#include "ChJs_system.h"
#include "ChJs_body.h"
#include "ChJs_marker.h"
#include "ChJs_link.h"
#include "ChJs_impact.h"
#include "physics/ChSystem.h"
#include "physics/ChGlobal.h"



namespace chrono 
{


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   SYSTEM class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec PSystem_props[] = {
    {"t",          0,	JSPROP_ENUMERATE},
    {"tol",        1,	JSPROP_ENUMERATE},
    {"Nbodies",    2,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Nlinks",     3,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Ncoords",    4,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Ndof",	   5,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Ndoc",	   6,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Nsysvars",   7,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Nredundancy",8,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Ncoords_w",  9,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Ndoc_w",     10,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Nsysvars_w", 11,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Ncontacts",  17,	JSPROP_ENUMERATE|JSPROP_READONLY},
    {"Gacc",       12,	JSPROP_ENUMERATE},
    {"step",       13,	JSPROP_ENUMERATE},
    {"step_min",   14,	JSPROP_ENUMERATE},
    {"step_max",   15,	JSPROP_ENUMERATE},
    //{"Y",		   16,	JSPROP_ENUMERATE},
    {0} // >= 18
};  

GET_JS_PARSE_BEGIN(PSystem_get, ChSystem*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->GetChTime()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->GetTol()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->GetNbodies()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->GetNlinks()) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp,this_data->GetNcoords()) )
	GET_JS_PROP (5,  chjs_from_double(cx,vp,this_data->GetNdof()) )
	GET_JS_PROP (6,  chjs_from_double(cx,vp,this_data->GetNdoc()) )
	GET_JS_PROP (7,  chjs_from_double(cx,vp,this_data->GetNsysvars()) )
	GET_JS_PROP (8,  chjs_from_double(cx,vp,this_data->GetNredundancy()) )
	GET_JS_PROP (9,  chjs_from_double(cx,vp,this_data->GetNcoords_w()) )
	GET_JS_PROP (10, chjs_from_double(cx,vp,this_data->GetNdoc_w()) )
	GET_JS_PROP (11, chjs_from_double(cx,vp,this_data->GetNsysvars_w()) )
	GET_JS_PROP (12, chjs_from_dataNEW(cx,vp, new Vector(this_data->Get_G_acc()), &chjs_Vector) )
	GET_JS_PROP (13, chjs_from_double(cx,vp,this_data->GetStep()) )
	GET_JS_PROP (14, chjs_from_double(cx,vp,this_data->GetStepMin()) )
	GET_JS_PROP (15, chjs_from_double(cx,vp,this_data->GetStepMax()) )
	//GET_JS_PROP (16, if (this_data->GetY()) chjs_from_data(cx,vp, this_data->GetY(),&chjs_Matrix); else chjs_from_bool(cx, vp, FALSE);)
	GET_JS_PROP (17, chjs_from_double(cx,vp,this_data->GetNcontacts()) )
GET_JS_PARSE_END
   
SET_JS_PARSE_BEGIN(PSystem_set, ChSystem*) 
	SET_JS_PROP (0,	&chjs_double, this_data->SetChTime(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->SetTol(chjs_to_double(cx,vp)) )
	SET_JS_PROP (12,&chjs_Vector, this_data->Set_G_acc(*(Vector*)chjs_to_data(cx,vp)) )
	SET_JS_PROP (13,&chjs_double, this_data->SetStep(chjs_to_double(cx,vp)) )
	SET_JS_PROP (14,&chjs_double, this_data->SetStepMin(chjs_to_double(cx,vp)) )
	SET_JS_PROP (15,&chjs_double, this_data->SetStepMax(chjs_to_double(cx,vp)) )
	//SET_JS_PROP (16,&chjs_Matrix, this_data->Update((ChMatrix<>*)chjs_to_data(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_FUNCTION(jsUpdate, ChSystem*, 0)
  this_data->Setup();
  this_data->Update();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsUpdateGeometry, ChSystem*, 0)
  this_data->Setup();
  this_data->UpdateExternalGeometry();
  *rval = JSVAL_VOID;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsAssembly, ChSystem*, 0)
  this_data->Setup();
  chjs_from_int(cx,rval,this_data->DoAssembly(ASS_POSITION|ASS_SPEED|ASS_ACCEL));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsAssemblyForces, ChSystem*, 0)
  this_data->Setup();
  chjs_from_int(cx,rval,this_data->DoFullAssembly());
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsDynamics, ChSystem*, 1)
  PARCHK(0, &chjs_double);
  double end_t = chjs_to_double(cx, argv+0);
  this_data->SetEndTime(end_t);
  chjs_from_int(cx,rval,this_data->DoEntireDynamics());
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsFrameDynamics, ChSystem*, 1)
  PARCHK(0, &chjs_double);
  chjs_from_int(cx,rval,this_data->DoFrameDynamics(chjs_to_double(cx, argv+0)));
DEF_JS_FUNEND 

DEF_JS_FUNCTION(jsObj, ChSystem*, 1)
  PARCHK(0, &chjs_string);
  ChBody* mbo = this_data->SearchBody(chjs_to_string(cx, argv+0));
  if (mbo) { chjs_from_data(cx,rval, mbo, &chjs_RBody); return JS_TRUE; }
  ChLink* mli = this_data->SearchLink(chjs_to_string(cx, argv+0));
  if (mli) { chjs_from_data(cx,rval, mli, chjs_cast_link(mli)); return JS_TRUE; }
  ChMarker* mma = this_data->SearchMarker(chjs_to_string(cx, argv+0));
  if (mma) { chjs_from_data(cx,rval, mma, &chjs_Marker); return JS_TRUE; }
  chjs_from_bool(cx,rval, false);
  return JS_TRUE; //return JS_FALSE;  ---no JS_FALSE otherwise 'slot out of index' error.
DEF_JS_FUNEND

//7
/*
static ChMatrixDynamic<> script_undo_matrix;
static double script_undo_time = 0;

DEF_JS_FUNCTION(jsUndoSave, ChSystem*, 0)
	this_data->Setup();
	this_data->Update_B2Y();
	if (this_data->GetY())
		if (this_data->GetY()->GetRows() >0)
		{
			script_undo_matrix.CopyFromMatrix(*this_data->GetY());
			script_undo_time = this_data->GetChTime();
		}
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsUndo, ChSystem*, 0)
	this_data->Setup();	
	if (this_data->GetY())
	{
		if (this_data->GetY()->GetRows() != script_undo_matrix.GetRows())
			JS_ReportError(cx,"Cannot perforn Undo action: last saved undo has a different size of system state vector.\n Did you delete or added some body in the meantime?");
		else
		{
			this_data->GetY()->CopyFromMatrix(script_undo_matrix);
			this_data->SetChTime(script_undo_time);
			this_data->Setup();
			this_data->Update();
		}
	}
	else
		JS_ReportError(cx,"Nothing in state vector of Chrono system. Nothing to undo.");
DEF_JS_FUNEND
*/








// ------- Method list -------------------------------
//
static JSFunctionSpec PSystem_methods[] = {
	{"Update",			jsUpdate,			0},
	{"UpdateGeometry",	jsUpdateGeometry,	0},
//	{"WireRefresh",		jsWireRefresh,		0},
	{"Assembly",		jsAssembly,			0},
	{"AssemblyForces",  jsAssemblyForces,	0},
	{"Dynamics",		jsDynamics,			0},
	{"FrameDynamics",	jsFrameDynamics,	0},
	{"Obj",				jsObj,				0},
//7	{"UndoSave",		jsUndoSave,			0},
//7	{"Undo",			jsUndo,				0},
    {0}
};



////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_PSystem = {
    "System", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  PSystem_get,		  PSystem_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};

JSClass* achjs_PSystem() {return &chjs_PSystem;} ;  // get address of static 'Javascript class' structure
  

////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_PSystem(JSContext* cx, JSObject* glob, JSObject* parent)
{
	// Define some global "utility" functions
	//JS_DefineFunctions(cx, glob, PSystem_glob_methods);

	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_PSystem,			// this class 
				NULL, 0,				// constructor fx and parameters
				PSystem_props, PSystem_methods, 
				NULL, NULL);
	return ret;
}



} // END_OF_NAMESPACE____
