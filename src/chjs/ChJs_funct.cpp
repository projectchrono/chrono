///////////////////////////////////////////////////
//
//   ChJs_funct.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChJs_utils.h"
#include "ChJs_math.h"
#include "ChJs_funct.h"
#include "ChJs_Engine.h"
#include "ChGlobalJS.h"
#include "ChFunctionJS.h"
#include "physics/ChGlobal.h"



namespace chrono
{



/////////////////////////////////////////////
////////// DOWNCAST UTILITY FUNCTION

JSClass* chjs_cast_funct(ChFunction* myfx)
{
	if (!myfx) return NULL;
	switch (myfx->Get_Type())
	{
	case FUNCT_CONST:
		return &chjs_Function;
	case FUNCT_RAMP:
		return &chjs_FunctionRamp;
	case FUNCT_SINE:
		return &chjs_FunctionSine;
	case FUNCT_SIGMA:
		return &chjs_FunctionSigma;
	case FUNCT_POLY:
		return &chjs_FunctionPoly;
	case FUNCT_CONSTACC:
		return &chjs_FunctionConstAcc;
	case FUNCT_POLY345:
		return &chjs_FunctionPoly345;
	case FUNCT_FILLET3:
		return &chjs_FunctionFillet3;
	case FUNCT_OPERATION:
		return &chjs_FunctionOperation;
	case FUNCT_RECORDER:
		return &chjs_FunctionRecorder;
	case FUNCT_SEQUENCE:
		return &chjs_FunctionSequence;
	case FUNCT_JSCRIPT:
		return &chjs_FunctionJscript;
	case FUNCT_DERIVE:
		return &chjs_FunctionDerive;
	case FUNCT_INTEGRATE:
		return &chjs_FunctionIntegrate;
	case FUNCT_MIRROR:
		return &chjs_FunctionMirror;
	case FUNCT_REPEAT:
		return &chjs_FunctionRepeat;

	default: return &chjs_Function;
	}
	return NULL;
}




//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Function_props[] = {
    {"C",       0,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(Function_get, ChFunction*)
	GET_JS_PROP (0, chjs_from_double(cx,vp,this_data->Get_yconst()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Function_set, ChFunction*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_yconst(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (Function_construct, ChFunction*)
  this_data = new ChFunction();
DEF_JS_BUILDEND

ChJS_FINALIZER (Function_finalize, ChFunction*)


DEF_JS_FUNCTION(jsGet_y, ChFunction*, 1)
  PARCHK(0, &chjs_double);
  chjs_from_double(cx, rval, this_data->Get_y(chjs_to_double(cx, argv+0)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsGet_y_dx, ChFunction*, 1)
  PARCHK(0, &chjs_double);
  chjs_from_double(cx, rval, this_data->Get_y_dx(chjs_to_double(cx, argv+0)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsGet_y_dxdx, ChFunction*, 1)
  PARCHK(0, &chjs_double);
  chjs_from_double(cx, rval, this_data->Get_y_dxdx(chjs_to_double(cx, argv+0)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMax, ChFunction*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  chjs_from_double(cx, rval, this_data->Compute_max( chjs_to_double(cx, argv+0),  chjs_to_double(cx, argv+1),  chjs_to_double(cx, argv+2), 0  ));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMin, ChFunction*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  chjs_from_double(cx, rval, this_data->Compute_min( chjs_to_double(cx, argv+0),  chjs_to_double(cx, argv+1),  chjs_to_double(cx, argv+2), 0  ));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMean, ChFunction*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  chjs_from_double(cx, rval, this_data->Compute_mean( chjs_to_double(cx, argv+0),  chjs_to_double(cx, argv+1),  chjs_to_double(cx, argv+2), 0  ));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsSqrMean, ChFunction*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  chjs_from_double(cx, rval, this_data->Compute_sqrmean( chjs_to_double(cx, argv+0),  chjs_to_double(cx, argv+1),  chjs_to_double(cx, argv+2), 0  ));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsInt, ChFunction*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  chjs_from_double(cx, rval, this_data->Compute_int( chjs_to_double(cx, argv+0),  chjs_to_double(cx, argv+1),  chjs_to_double(cx, argv+2), 0  ));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsOptVarTree, ChFunction*, 0)
  ChList<chjs_propdata> mylist;
  this_data->MakeOptVariableTree(&mylist);
  chjs_print_objtree(cx, 0, &mylist);
  mylist.KillAll();
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsOptVarList, ChFunction*, 0)
  ChList<chjs_propdata> mytree;
  ChList<chjs_fullnamevar> mylist;
  this_data->MakeOptVariableTree(&mytree);
  ChFunction::VariableTreeToFullNameVar(&mytree,&mylist);
  ChNode<chjs_fullnamevar>* mynode = mylist.GetHead();
  while (mynode)
  {

	  CHGLOBALS_JS().chjsEngine->chjs_Print(mynode->data->propname);
	  mynode= mynode->next;
  }
  mytree.KillAll();
  mylist.KillAll();
DEF_JS_FUNEND



DEF_JS_FUNCTION(jsOptVarToVect, ChFunction*, 1)
  PARCHK(0, &chjs_Matrix);
  chjs_from_int(cx, rval, ChFunctionOptvarToolsJS::OptVariablesToVector(this_data, (ChMatrix<>*)chjs_to_data(cx, argv+0), 0 ) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsVectToOptVar, ChFunction*, 1)
  PARCHK(0, &chjs_Matrix);
  chjs_from_int(cx, rval, ChFunctionOptvarToolsJS::VectorToOptVariables(this_data, (ChMatrix<>*)chjs_to_data(cx, argv+0), 0 ) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsOptVarCount, ChFunction*, 0)
  chjs_from_int(cx, rval, this_data->OptVariableCount());
DEF_JS_FUNEND

// ------- Method list -------------------------------
//
static JSFunctionSpec Function_methods[] = {
	{"y",			jsGet_y,			0},
	{"y_dx",		jsGet_y_dx,			0},
	{"y_dxdx",		jsGet_y_dxdx,		0},
	{"Max",			jsMax,				0},
	{"Min",			jsMin,				0},
	{"Mean",		jsMean,				0},
	{"SqrMean",		jsSqrMean,			0},
	{"Int",			jsInt,				0},
	{"OptVarTree",  jsOptVarTree,		0},
	{"OptVarList",  jsOptVarList,		0},
	{"OptVarCount", jsOptVarCount,		0},
	{"OptVarToVect",jsOptVarToVect,		0},
	{"VectToOptVar",jsVectToOptVar,		0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Function = {
    "ChFunction", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Function_get,	  Function_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   Function_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_Function(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_Function,			// this class
				Function_construct, 0,	// constructor fx and parameters
				Function_props, Function_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION RAMP class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionRamp_props[] = {
    {"C",       0,	JSPROP_ENUMERATE},
    {"ang",     1,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionRamp_get, ChFunction_Ramp*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->Get_y0()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_ang()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionRamp_set, ChFunction_Ramp*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_y0(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_ang(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionRamp_construct, ChFunction_Ramp*)
  this_data = new ChFunction_Ramp();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionRamp_finalize, ChFunction_Ramp*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionRamp_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionRamp = {
    "ChFunctionRamp", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionRamp_get,	  FunctionRamp_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   FunctionRamp_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionRamp(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionRamp,			// this class
				FunctionRamp_construct, 0,		// constructor fx and parameters
				FunctionRamp_props, FunctionRamp_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION SINE class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionSine_props[] = {
    {"amp",       0,	JSPROP_ENUMERATE},
    {"phase",     1,	JSPROP_ENUMERATE},
    {"freq",	  2,	JSPROP_ENUMERATE},
    {"w",	      3,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionSine_get, ChFunction_Sine*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->Get_amp()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_phase()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->Get_freq()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->Get_w()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionSine_set, ChFunction_Sine*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_amp(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_phase(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,	&chjs_double, this_data->Set_freq(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,	&chjs_double, this_data->Set_w(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionSine_construct, ChFunction_Sine*)
  this_data = new ChFunction_Sine();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionSine_finalize, ChFunction_Sine*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionSine_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionSine = {
    "ChFunctionSine", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionSine_get,	  FunctionSine_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   FunctionSine_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionSine(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionSine,			// this class
				FunctionSine_construct, 0,				// constructor fx and parameters
				FunctionSine_props, FunctionSine_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION SIGMA class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionSigma_props[] = {
    {"start",       0,	JSPROP_ENUMERATE},
    {"end",			1,	JSPROP_ENUMERATE},
    {"amp",			2,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionSigma_get, ChFunction_Sigma*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->Get_start()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_end()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->Get_amp()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionSigma_set, ChFunction_Sigma*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_start(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_end(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,	&chjs_double, this_data->Set_amp(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionSigma_construct, ChFunction_Sigma*)
  this_data = new ChFunction_Sigma();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionSigma_finalize, ChFunction_Sigma*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionSigma_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionSigma = {
    "ChFunctionSigma", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionSigma_get,	  FunctionSigma_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,    FunctionSigma_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionSigma(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionSigma,			// this class
				FunctionSigma_construct, 0,				// constructor fx and parameters
				FunctionSigma_props, FunctionSigma_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION POLY class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionPoly_props[] = {
    {"order",       0,	JSPROP_ENUMERATE},
    {"C0",	        1,	JSPROP_ENUMERATE},
    {"C1",	        2,	JSPROP_ENUMERATE},
    {"C2",	        3,	JSPROP_ENUMERATE},
    {"C3",	        4,	JSPROP_ENUMERATE},
    {"C4",	        5,	JSPROP_ENUMERATE},
    {"C5",	        6,	JSPROP_ENUMERATE},
    {"C6",	        7,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionPoly_get, ChFunction_Poly*)
	GET_JS_PROP (0,  chjs_from_int(cx,vp,this_data->Get_order()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_coeff(0)) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->Get_coeff(1)) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->Get_coeff(2)) )
	GET_JS_PROP (4,  chjs_from_double(cx,vp,this_data->Get_coeff(3)) )
	GET_JS_PROP (5,  chjs_from_double(cx,vp,this_data->Get_coeff(4)) )
	GET_JS_PROP (6,  chjs_from_double(cx,vp,this_data->Get_coeff(5)) )
	GET_JS_PROP (7,  chjs_from_double(cx,vp,this_data->Get_coeff(6)) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionPoly_set, ChFunction_Poly*)
	SET_JS_PROP (0,	&chjs_int, this_data->Set_order(chjs_to_int(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_coeff(chjs_to_double(cx,vp),0) )
	SET_JS_PROP (2,	&chjs_double, this_data->Set_coeff(chjs_to_double(cx,vp),1) )
	SET_JS_PROP (3,	&chjs_double, this_data->Set_coeff(chjs_to_double(cx,vp),2) )
	SET_JS_PROP (4,	&chjs_double, this_data->Set_coeff(chjs_to_double(cx,vp),3) )
	SET_JS_PROP (5,	&chjs_double, this_data->Set_coeff(chjs_to_double(cx,vp),4) )
	SET_JS_PROP (6,	&chjs_double, this_data->Set_coeff(chjs_to_double(cx,vp),5) )
	SET_JS_PROP (7,	&chjs_double, this_data->Set_coeff(chjs_to_double(cx,vp),6) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionPoly_construct, ChFunction_Poly*)
  this_data = new ChFunction_Poly();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionPoly_finalize, ChFunction_Poly*)

DEF_JS_FUNCTION(jsSetCoeff, ChFunction_Poly*, 2)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_int);
  this_data->Set_coeff(chjs_to_double(cx, argv+0), chjs_to_int(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsGetCoeff, ChFunction_Poly*, 1)
  PARCHK(0, &chjs_int);
  chjs_from_double(cx, rval, this_data->Get_coeff(chjs_to_int(cx, argv+0))  ) ;
DEF_JS_FUNEND


// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionPoly_methods[] = {
	{"set_coeff",			jsSetCoeff,			0},
	{"get_coeff",			jsGetCoeff,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionPoly = {
    "ChFunctionPoly", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionPoly_get,	  FunctionPoly_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   FunctionPoly_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionPoly(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionPoly,			// this class
				FunctionPoly_construct, 0,				// constructor fx and parameters
				FunctionPoly_props, FunctionPoly_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION CONSTACC class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionConstAcc_props[] = {
    {"h",			0,	JSPROP_ENUMERATE},
    {"end",			1,	JSPROP_ENUMERATE},
    {"aw",			2,	JSPROP_ENUMERATE},
    {"av",			3,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionConstAcc_get, ChFunction_ConstAcc*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->Get_h()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_end()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->Get_aw()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->Get_av()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionConstAcc_set, ChFunction_ConstAcc*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_h(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_end(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,	&chjs_double, this_data->Set_aw(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,	&chjs_double, this_data->Set_av(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//
DEF_JS_BUILDER (FunctionConstAcc_construct, ChFunction_ConstAcc*)
  this_data = new ChFunction_ConstAcc();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionConstAcc_finalize, ChFunction_ConstAcc*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionConstAcc_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionConstAcc = {
    "ChFunctionConstAcc", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionConstAcc_get,	  FunctionConstAcc_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,    FunctionConstAcc_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionConstAcc(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionConstAcc,			// this class
				FunctionConstAcc_construct, 0,				// constructor fx and parameters
				FunctionConstAcc_props, FunctionConstAcc_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION POLY345 class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionPoly345_props[] = {
    {"h",			0,	JSPROP_ENUMERATE},
    {"end",			1,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionPoly345_get, ChFunction_Poly345*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->Get_h()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_end()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionPoly345_set, ChFunction_Poly345*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_h(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_end(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionPoly345_construct, ChFunction_Poly345*)
  this_data = new ChFunction_Poly345();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionPoly345_finalize, ChFunction_Poly345*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionPoly345_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionPoly345 = {
    "ChFunctionPoly345", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionPoly345_get,	  FunctionPoly345_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,      FunctionPoly345_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionPoly345(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionPoly345,			// this class
				FunctionPoly345_construct, 0,				// constructor fx and parameters
				FunctionPoly345_props, FunctionPoly345_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION FILLET3 class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionFillet3_props[] = {
    {"y1",			0,	JSPROP_ENUMERATE},
    {"y2",			1,	JSPROP_ENUMERATE},
    {"dy1",			2,	JSPROP_ENUMERATE},
    {"dy2",			3,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionFillet3_get, ChFunction_Fillet3*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->Get_y1()) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_y2()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->Get_dy1()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->Get_dy2()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionFillet3_set, ChFunction_Fillet3*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_y1(chjs_to_double(cx,vp)) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_y2(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,	&chjs_double, this_data->Set_dy1(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,	&chjs_double, this_data->Set_dy2(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionFillet3_construct, ChFunction_Fillet3*)
  this_data = new ChFunction_Fillet3();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionFillet3_finalize, ChFunction_Fillet3*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionFillet3_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionFillet3 = {
    "ChFunctionFillet3", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionFillet3_get,	  FunctionFillet3_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   FunctionFillet3_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionFillet3(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionFillet3,			// this class
				FunctionFillet3_construct, 0,				// constructor fx and parameters
				FunctionFillet3_props, FunctionFillet3_methods,
				NULL, NULL);
	return ret;
}




//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION OPERATION class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionOperation_props[] = {
    {"fa",			0,	JSPROP_ENUMERATE},
    {"fb",			1,	JSPROP_ENUMERATE},
    {"op_type",		2,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionOperation_get, ChFunction_Operation*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp,this_data->Get_fa(), chjs_cast_funct(this_data->Get_fa())) )
	GET_JS_PROP (1,  chjs_from_data(cx,vp,this_data->Get_fb(), chjs_cast_funct(this_data->Get_fb())) )
	GET_JS_PROP (2,  chjs_from_int(cx,vp,this_data->Get_optype()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionOperation_set, ChFunction_Operation*)
	SET_JS_PROP (0,	&chjs_Function, this_data->Set_fa(((ChFunction*)chjs_to_data(cx,vp))->new_Duplicate() ) )
	SET_JS_PROP (1,	&chjs_Function, this_data->Set_fb(((ChFunction*)chjs_to_data(cx,vp))->new_Duplicate() ) )
	SET_JS_PROP (2,	&chjs_int, this_data->Set_optype(chjs_to_int(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionOperation_construct, ChFunction_Operation*)
  this_data = new ChFunction_Operation();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionOperation_finalize, ChFunction_Operation*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionOperation_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionOperation = {
    "ChFunctionOperation", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionOperation_get,	  FunctionOperation_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,			  FunctionOperation_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionOperation(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionOperation,			// this class
				FunctionOperation_construct, 0,				// constructor fx and parameters
				FunctionOperation_props, FunctionOperation_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION RECORDER class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionRecorder_props[] = {
    //{"order",       0,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionRecorder_get, ChFunction_Recorder*)
	//GET_JS_PROP (0,  chjs_from_int(cx,vp,this_data->Get_order()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionRecorder_set, ChFunction_Recorder*)
	//SET_JS_PROP (0,	&chjs_int, this_data->Set_order(chjs_to_int(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionRecorder_construct, ChFunction_Recorder*)
  this_data = new ChFunction_Recorder();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionRecorder_finalize, ChFunction_Recorder*)

DEF_JS_FUNCTION(jsAddPoint, ChFunction_Recorder*, 2)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  this_data->AddPoint(chjs_to_double(cx, argv+0), chjs_to_double(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsAddPointClean, ChFunction_Recorder*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  this_data->AddPointClean(chjs_to_double(cx, argv+0), chjs_to_double(cx, argv+1), chjs_to_double(cx, argv+2));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsReset, ChFunction_Recorder*, 0)
  this_data->Reset();
DEF_JS_FUNEND


// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionRecorder_methods[] = {
	{"add_point",			jsAddPoint,			0},
	{"add_point_clean",		jsAddPointClean,    0},
	{"reset",				jsReset,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionRecorder = {
    "ChFunctionRecorder", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionRecorder_get,	  FunctionRecorder_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,    FunctionRecorder_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionRecorder(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionRecorder,			// this class
				FunctionRecorder_construct, 0,				// constructor fx and parameters
				FunctionRecorder_props, FunctionRecorder_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION NODE  class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionNode_props[] = {
    {"fx",		       0,	JSPROP_ENUMERATE},
	{"duration",       1,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionNode_get, ChFseqNode*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp,this_data->fx, chjs_cast_funct(this_data->fx) ) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->duration) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionNode_set, ChFseqNode*)
	SET_JS_PROP (0,	&chjs_Function, this_data->fx = ((ChFunction*)chjs_to_data(cx,vp))  )
	SET_JS_PROP (1,	&chjs_double, this_data->SetDuration(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionNode_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionNode = {
    "ChFunctionNode", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionNode_get,	  FunctionNode_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   JS_FinalizeStub,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionNode(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionNode,			// this class
				NULL, 0,				// constructor fx and parameters
				FunctionNode_props, FunctionNode_methods,
				NULL, NULL);
	return ret;
}





//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION SEQUENCE class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionSequence_props[] = {
    {"start",       0,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionSequence_get, ChFunction_Sequence*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->Get_start()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionSequence_set, ChFunction_Sequence*)
	SET_JS_PROP (0,	&chjs_double, this_data->Set_start(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionSequence_construct, ChFunction_Sequence*)
  this_data = new ChFunction_Sequence();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionSequence_finalize, ChFunction_Sequence*)

DEF_JS_FUNCTION_NPARS(jsInsert, ChFunction_Sequence*)
switch(argc){
case 7:
  PARCHK(0, &chjs_Function);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  PARCHK(3, &chjs_boolean);
  PARCHK(4, &chjs_boolean);
  PARCHK(5, &chjs_boolean);
  PARCHK(6, &chjs_int);
  chjs_from_int(cx, rval, this_data->InsertFunct(
						((ChFunction*)chjs_to_data(cx, argv+0))->new_Duplicate(),
						chjs_to_double(cx, argv+1) ,
						chjs_to_double(cx, argv+2) ,
						chjs_to_bool(cx, argv+3) ,
						chjs_to_bool(cx, argv+4) ,
						chjs_to_bool(cx, argv+5) ,
						chjs_to_int(cx, argv+6) ) );
  break;
case 3:
  PARCHK(0, &chjs_Function);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_int);
  chjs_from_int(cx, rval, this_data->InsertFunct(
						((ChFunction*)chjs_to_data(cx, argv+0))->new_Duplicate(),
						chjs_to_double(cx, argv+1) ,
						1.0, false, false,false,
						chjs_to_int(cx, argv+2) ) );
  break;
default:
  JS_ReportError(cx, "Function needs 7 or 3 parameters."); return JS_FALSE;
  break;
}
DEF_JS_FUNEND

DEF_JS_FUNCTION_NPARS(jsAppend, ChFunction_Sequence*)
  if (argc<2)
	{JS_ReportError(cx, "Function needs pairs of 'function, duration, ..' as parameters."); return JS_FALSE;}
  int icouple=0;
  while ((icouple+1)<(int)argc)
  {
	PARCHK(0+icouple, &chjs_Function);
	PARCHK(1+icouple, &chjs_double);
	chjs_from_int(cx, rval, this_data->InsertFunct(
						((ChFunction*)chjs_to_data(cx, argv+icouple+0))->new_Duplicate(),
						chjs_to_double(cx, argv+icouple+1) ,
						1.0, false, false,false, -1 ) );
	icouple +=2;
  }
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsKillFunct, ChFunction_Sequence*, 1)
  PARCHK(0, &chjs_int);
  chjs_from_int(cx, rval, this_data->KillFunct(chjs_to_int(cx, argv+0)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsGetNthFunction, ChFunction_Sequence*, 1)
  PARCHK(0, &chjs_int);
  ChFunction* mf =  this_data->GetNthFunction(chjs_to_int(cx, argv+0));
  chjs_from_data(cx, rval, mf, chjs_cast_funct(mf));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsGetNthNode, ChFunction_Sequence*, 1)
  PARCHK(0, &chjs_int);
  chjs_from_data(cx, rval, this_data->GetNthNode(chjs_to_int(cx, argv+0)), &chjs_FunctionNode );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsGetNthDuration, ChFunction_Sequence*, 1)
  PARCHK(0, &chjs_int);
  chjs_from_double(cx, rval, this_data->GetNthDuration(chjs_to_int(cx, argv+0)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsSeqSetup, ChFunction_Sequence*, 0)
  this_data->Setup();
DEF_JS_FUNEND


// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionSequence_methods[] = {
	{"insert",				jsInsert,			0},
	{"kill_fn",				jsKillFunct,		0},
	{"get_fn",				jsGetNthFunction,	0},
	{"node_n",				jsGetNthNode,		0},
	{"duration_n",			jsGetNthDuration,	0},
	{"setup",				jsSeqSetup,			0},
	{"append",				jsAppend,			0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionSequence = {
    "ChFunctionSequence", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionSequence_get,	  FunctionSequence_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   FunctionSequence_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionSequence(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionSequence,			// this class
				FunctionSequence_construct, 0,				// constructor fx and parameters
				FunctionSequence_props, FunctionSequence_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION JSCRIPT class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionJscript_props[] = {
    {"command",  0,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionJscript_get, ChFunction_Jscript*)
	GET_JS_PROP (0,  chjs_from_string(cx,vp,this_data->Get_Command()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionJscript_set, ChFunction_Jscript*)
	SET_JS_PROP (0,	&chjs_string, this_data->Set_Command(chjs_to_string(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionJscript_construct, ChFunction_Jscript*)
  this_data = new ChFunction_Jscript();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionJscript_finalize, ChFunction_Jscript*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionJscript_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionJscript = {
    "ChFunctionJscript", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionJscript_get,	  FunctionJscript_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,		      FunctionJscript_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionJscript(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionJscript,			// this class
				FunctionJscript_construct, 0,				// constructor fx and parameters
				FunctionJscript_props, FunctionJscript_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION DERIVE class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionDerive_props[] = {
    {"fa",			0,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionDerive_get, ChFunction_Derive*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp,this_data->Get_fa(), chjs_cast_funct(this_data->Get_fa())) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionDerive_set, ChFunction_Derive*)
	SET_JS_PROP (0,	&chjs_Function, this_data->Set_fa(((ChFunction*)chjs_to_data(cx,vp))->new_Duplicate() ) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionDerive_construct, ChFunction_Derive*)
  this_data = new ChFunction_Derive();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionDerive_finalize, ChFunction_Derive*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionDerive_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionDerive = {
    "ChFunctionDerive", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionDerive_get,	  FunctionDerive_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,			  FunctionDerive_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionDerive(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionDerive,			// this class
				FunctionDerive_construct, 0,				// constructor fx and parameters
				FunctionDerive_props, FunctionDerive_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION INTEGRATE class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionIntegrate_props[] = {
    {"fa",			0,	JSPROP_ENUMERATE},
	{"C_start",     1,	JSPROP_ENUMERATE},
	{"x_start",     2,	JSPROP_ENUMERATE},
	{"x_end",       3,	JSPROP_ENUMERATE},
	{"num_samples", 4,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionIntegrate_get, ChFunction_Integrate*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp,this_data->Get_fa(), chjs_cast_funct(this_data->Get_fa())) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_C_start()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->Get_x_start()) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->Get_x_end()) )
	GET_JS_PROP (4,  chjs_from_int(cx,vp,this_data->Get_num_samples()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionIntegrate_set, ChFunction_Integrate*)
	SET_JS_PROP (0,	&chjs_Function, this_data->Set_fa(((ChFunction*)chjs_to_data(cx,vp))->new_Duplicate() ) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_C_start(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,	&chjs_double, this_data->Set_x_start(chjs_to_double(cx,vp)) )
	SET_JS_PROP (3,	&chjs_double, this_data->Set_x_end(chjs_to_double(cx,vp)) )
	SET_JS_PROP (4,	&chjs_int, this_data->Set_num_samples(chjs_to_int(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionIntegrate_construct, ChFunction_Integrate*)
  this_data = new ChFunction_Integrate();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionIntegrate_finalize, ChFunction_Integrate*)

DEF_JS_FUNCTION(jsComputeIntegral, ChFunction_Integrate*, 1)
  this_data->ComputeIntegral();
DEF_JS_FUNEND

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionIntegrate_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionIntegrate = {
    "ChFunctionIntegrate", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionIntegrate_get,	  FunctionIntegrate_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,			  FunctionIntegrate_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionIntegrate(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionIntegrate,			// this class
				FunctionIntegrate_construct, 0,				// constructor fx and parameters
				FunctionIntegrate_props, FunctionIntegrate_methods,
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION MIRROR class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionMirror_props[] = {
    {"fa",			0,	JSPROP_ENUMERATE},
	{"mirror_axis", 1,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionMirror_get, ChFunction_Mirror*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp,this_data->Get_fa(), chjs_cast_funct(this_data->Get_fa())) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_mirror_axis()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionMirror_set, ChFunction_Mirror*)
	SET_JS_PROP (0,	&chjs_Function, this_data->Set_fa(((ChFunction*)chjs_to_data(cx,vp))->new_Duplicate() ) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_mirror_axis(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionMirror_construct, ChFunction_Mirror*)
  this_data = new ChFunction_Mirror();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionMirror_finalize, ChFunction_Mirror*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionMirror_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionMirror = {
    "ChFunctionMirror", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionMirror_get,	  FunctionMirror_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,			  FunctionMirror_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionMirror(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionMirror,			// this class
				FunctionMirror_construct, 0,				// constructor fx and parameters
				FunctionMirror_props, FunctionMirror_methods,
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   FUNCTION REPEAT class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec FunctionRepeat_props[] = {
    {"fa",			0,	JSPROP_ENUMERATE},
	{"window_start",1,	JSPROP_ENUMERATE},
	{"window_length",2,	JSPROP_ENUMERATE},
    {0}
};

GET_JS_PARSE_BEGIN(FunctionRepeat_get, ChFunction_Repeat*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp,this_data->Get_fa(), chjs_cast_funct(this_data->Get_fa())) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->Get_window_start()) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->Get_window_length()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(FunctionRepeat_set, ChFunction_Repeat*)
	SET_JS_PROP (0,	&chjs_Function, this_data->Set_fa(((ChFunction*)chjs_to_data(cx,vp))->new_Duplicate() ) )
	SET_JS_PROP (1,	&chjs_double, this_data->Set_window_start(chjs_to_double(cx,vp)) )
	SET_JS_PROP (2,	&chjs_double, this_data->Set_window_length(chjs_to_double(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (FunctionRepeat_construct, ChFunction_Repeat*)
  this_data = new ChFunction_Repeat();
DEF_JS_BUILDEND

ChJS_FINALIZER (FunctionRepeat_finalize, ChFunction_Repeat*)

// ------- Method list -------------------------------
//
static JSFunctionSpec FunctionRepeat_methods[] = {
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_FunctionRepeat = {
    "ChFunctionRepeat", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  FunctionRepeat_get,	  FunctionRepeat_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,			  FunctionRepeat_finalize,
};


////////////////////////////////////////////////////////////////////
//
// INITIALIZATION
//
JSObject* ChJS_InitClass_FunctionRepeat(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob,
				parent,					// parent prototype (parent class)
				&chjs_FunctionRepeat,			// this class
				FunctionRepeat_construct, 0,				// constructor fx and parameters
				FunctionRepeat_props, FunctionRepeat_methods,
				NULL, NULL);
	return ret;
}



} // END_OF_NAMESPACE____
