///////////////////////////////////////////////////
//  
//   ChJs_math.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChJs_utils.h"
#include "ChJs_math.h"
#include "ChJs_geometry.h"
#include "ChJs_Engine.h"
#include "ChGlobalJS.h"
#include "core/ChMath.h" 
#include "core/ChLinearAlgebra.h" 
#include "physics/ChIterative.h"
#include "physics/ChGlobal.h"


namespace chrono 
{


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   VECTOR class 
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Vector_props[] = {
    {"x",          0,	JSPROP_ENUMERATE},
    {"y",          1,	JSPROP_ENUMERATE},
    {"z",          2,	JSPROP_ENUMERATE},
    {0}
}; 

GET_JS_PARSE_BEGIN(Vector_get, Vector*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->x) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->y) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->z) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Vector_set, Vector*)
	SET_JS_PROP (0,	&chjs_double, this_data->x = chjs_to_double(cx,vp) )
	SET_JS_PROP (1,	&chjs_double, this_data->y = chjs_to_double(cx,vp) )
	SET_JS_PROP (2,	&chjs_double, this_data->z = chjs_to_double(cx,vp) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//
DEF_JS_BUILDER (Vector_construct, Vector*) 
	if (argc==0)
		this_data = new Vector(VNULL);
	if (argc==3)
	{
		this_data = new Vector(); 
		this_data->x=chjs_to_double(cx, argv+0); this_data->y=chjs_to_double(cx, argv+1); this_data->z=chjs_to_double(cx, argv+2);
	};
DEF_JS_BUILDEND

ChJS_FINALIZER (Vector_finalize, Vector*)

   
 
DEF_JS_FUNCTION(jsVlenght, Vector*, 0)
  chjs_from_double(cx, rval, (Vlenght(*this_data)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsVnorm, Vector*, 1)
  PARCHK(0, &chjs_Vector);
  *this_data = Vnorm(*(Vector*)chjs_to_data(cx, argv+0));
DEF_JS_FUNEND
 
DEF_JS_FUNCTION(jsVmul, Vector*, 1)
  PARCHK(0, &chjs_double);
  *this_data = Vmul(*this_data, chjs_to_double(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsVcross, Vector*, 2)
  PARCHK(0, &chjs_Vector);
  PARCHK(1, &chjs_Vector);
  *this_data = Vcross(*(Vector*)chjs_to_data(cx, argv+0), *(Vector*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsVdot, Vector*, 2)
  PARCHK(0, &chjs_Vector);
  PARCHK(1, &chjs_Vector);
  chjs_from_double(cx, rval,  Vdot(*(Vector*)chjs_to_data(cx, argv+0), *(Vector*)chjs_to_data(cx, argv+1)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsVadd, Vector*, 2)
  PARCHK(0, &chjs_Vector);
  PARCHK(1, &chjs_Vector);
  *this_data = Vadd(*(Vector*)chjs_to_data(cx, argv+0), *(Vector*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND
 
DEF_JS_FUNCTION(jsVsub, Vector*, 2)
  PARCHK(0, &chjs_Vector);
  PARCHK(1, &chjs_Vector);
  *this_data = Vsub(*(Vector*)chjs_to_data(cx, argv+0), *(Vector*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsVset, Vector*, 3)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  this_data->x= chjs_to_double(cx, argv+0);   this_data->y= chjs_to_double(cx, argv+1);   this_data->z= chjs_to_double(cx, argv+2);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsVToString, Vector*, 0)
  static char num[60];
  sprintf(num, "%g	%g	%g ", this_data->x, this_data->y, this_data->z  ); 
  chjs_from_string(cx,rval, num);
DEF_JS_FUNEND

// ------- Method list -------------------------------
//
static JSFunctionSpec Vector_methods[] = {
	{"Vlength",			jsVlenght,			0},
	{"Vnorm",			jsVnorm,			0},
	{"Vmul",			jsVmul,				0},
	{"Vcross",			jsVcross,			0},
	{"Vdot",			jsVdot,				0},
	{"Vadd",			jsVadd,				0},
	{"Vsub",			jsVsub,				0},
	{"Vadd",			jsVadd,				0},
	{"Vset",			jsVset,				0},
	{"toString",		jsVToString,		0},
    {0}
};



////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Vector = {
    "Vector", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Vector_get,		  Vector_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   Vector_finalize,
};
JSClass* achjs_Vector() {return &chjs_Vector;}

    
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Vector(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Vector,			// this class 
				Vector_construct, 0,	// constructor fx and parameters
				Vector_props, Vector_methods, 
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   QUATERNION class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Quaternion_props[] = {
    {"e0",          0,	JSPROP_ENUMERATE},
    {"e1",          1,	JSPROP_ENUMERATE},
    {"e2",          2,	JSPROP_ENUMERATE},
    {"e3",          3,	JSPROP_ENUMERATE},
    {0}
}; 

GET_JS_PARSE_BEGIN(Quaternion_get, Quaternion*)
	GET_JS_PROP (0,  chjs_from_double(cx,vp,this_data->e0) )
	GET_JS_PROP (1,  chjs_from_double(cx,vp,this_data->e1) )
	GET_JS_PROP (2,  chjs_from_double(cx,vp,this_data->e2) )
	GET_JS_PROP (3,  chjs_from_double(cx,vp,this_data->e3) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Quaternion_set, Quaternion*)
	SET_JS_PROP (0,	&chjs_double, this_data->e0 = chjs_to_double(cx,vp) )
	SET_JS_PROP (1,	&chjs_double, this_data->e1 = chjs_to_double(cx,vp) )
	SET_JS_PROP (2,	&chjs_double, this_data->e2 = chjs_to_double(cx,vp) )
	SET_JS_PROP (3,	&chjs_double, this_data->e3 = chjs_to_double(cx,vp) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (Quaternion_construct, Quaternion*)
  if (argc==0)
	this_data = new Quaternion(QNULL);
  if (argc==4)
  {
	this_data = new Quaternion(); 
	this_data->e0=chjs_to_double(cx, argv+0); this_data->e1=chjs_to_double(cx, argv+1); this_data->e2=chjs_to_double(cx, argv+2); this_data->e3=chjs_to_double(cx, argv+3);
  }
DEF_JS_BUILDEND

ChJS_FINALIZER (Quaternion_finalize, Quaternion*)



DEF_JS_FUNCTION(jsQlenght, Quaternion*, 0)
  chjs_from_double(cx, rval, (Qlenght(*this_data)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQnorm, Quaternion*, 1)
  PARCHK(0, &chjs_Quaternion);
  *this_data = Qnorm(*(Quaternion*)chjs_to_data(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQscale, Quaternion*, 1)
  PARCHK(0, &chjs_double);
  *this_data = Qscale(*this_data, chjs_to_double(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQcross, Quaternion*, 2)
  PARCHK(0, &chjs_Quaternion);
  PARCHK(1, &chjs_Quaternion);
  *this_data = Qcross(*(Quaternion*)chjs_to_data(cx, argv+0), *(Quaternion*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQadd, Quaternion*, 2)
  PARCHK(0, &chjs_Quaternion);
  PARCHK(1, &chjs_Quaternion);
  *this_data = Qadd(*(Quaternion*)chjs_to_data(cx, argv+0), *(Quaternion*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND
 
DEF_JS_FUNCTION(jsQsub, Quaternion*, 2)
  PARCHK(0, &chjs_Quaternion);
  PARCHK(1, &chjs_Quaternion);
  *this_data = Qsub(*(Quaternion*)chjs_to_data(cx, argv+0), *(Quaternion*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQset, Quaternion*, 4)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  PARCHK(2, &chjs_double);
  PARCHK(3, &chjs_double);
  this_data->e0= chjs_to_double(cx, argv+0);   this_data->e1= chjs_to_double(cx, argv+1);
  this_data->e2= chjs_to_double(cx, argv+2);   this_data->e3= chjs_to_double(cx, argv+3);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQtoEulero, Quaternion*, 0)
  ChMatrix33<> temp_coord;
  temp_coord.Set_A_quaternion(*this_data);
  chjs_from_dataNEW(cx,rval, new Vector(temp_coord.Get_A_Eulero()), &chjs_Vector);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQtoCardano, Quaternion*, 0)
  ChMatrix33<> temp_coord;
  temp_coord.Set_A_quaternion(*this_data);
  chjs_from_dataNEW(cx,rval, new Vector(temp_coord.Get_A_Cardano()), &chjs_Vector);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQtoRodriguez, Quaternion*, 0)
  ChMatrix33<> temp_coord;
  temp_coord.Set_A_quaternion(*this_data);
  chjs_from_dataNEW(cx,rval, new Vector(temp_coord.Get_A_Rodriguez()), &chjs_Vector);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQfromAngAxis, Quaternion*, 2)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_Vector);
  *this_data = Q_from_AngAxis(chjs_to_double(cx, argv+0), *(Vector*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsQToString, Quaternion*, 0)
  static char num[60];
  sprintf(num, "%g	%g	%g	%g ", this_data->e0, this_data->e1, this_data->e2, this_data->e3  ); 
  chjs_from_string(cx,rval, num);
DEF_JS_FUNEND



// ------- Method list -------------------------------
//
static JSFunctionSpec Quaternion_methods[] = {
	{"Qlenght",			jsQlenght,			0},
	{"Qnorm",			jsQnorm,			0},
	{"Qscale",			jsQscale,			0},
	{"Qcross",			jsQcross,			0},
	{"Qadd",			jsQadd,				0},
	{"Qsub",			jsQsub,				0},
	{"Qset",			jsQset,				0},
	{"QtoEulero",		jsQtoEulero,		0},
	{"QtoCardano",		jsQtoCardano,		0},
	{"QtoRodriguez",	jsQtoRodriguez,		0},
	{"QfromAngAxis",	jsQfromAngAxis,		0},
	{"toString",		jsQToString,		0},
    {0}
};


////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Quaternion = {
    "Quaternion", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Quaternion_get,	  Quaternion_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   Quaternion_finalize,
};

JSClass* achjs_Quaternion() {return &chjs_Quaternion;}
  

////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Quaternion(JSContext* cx, JSObject* glob, JSObject* parent)
{

	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Quaternion,		// this class 
				Quaternion_construct, 0,// constructor fx and parameters
				Quaternion_props, Quaternion_methods, 
				NULL, NULL);
	return ret;
}



//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   COORDSYS class
//

////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Coordsys_props[] = {
    {"pos",          0,	JSPROP_ENUMERATE},
    {"rot",          1,	JSPROP_ENUMERATE},
    {0}
}; 

GET_JS_PARSE_BEGIN(Coordsys_get, Coordsys*)
	GET_JS_PROP (0,  chjs_from_data(cx,vp, (void*)&this_data->pos, &chjs_Vector) )
	GET_JS_PROP (1,  chjs_from_data(cx,vp, (void*)&this_data->rot, &chjs_Quaternion) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Coordsys_set, Coordsys*)
	SET_JS_PROP (0,	&chjs_Vector,		this_data->pos = *(Vector*)chjs_to_data(cx,vp) )
	SET_JS_PROP (1,	&chjs_Quaternion,	this_data->rot = *(Quaternion*)chjs_to_data(cx,vp) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (Coordsys_construct, Coordsys*)
  this_data = new Coordsys(CSYSNULL);
DEF_JS_BUILDEND

ChJS_FINALIZER (Coordsys_finalize, Coordsys*)



// ------- Method list -------------------------------
//
static JSFunctionSpec Coordsys_methods[] = {
    {0}
};

////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Coordsys = {
    "Coordsys", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Coordsys_get,	  Coordsys_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   Coordsys_finalize,
};

JSClass* achjs_Coordsys() {return &chjs_Coordsys;}

  
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Coordsys(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Coordsys,			// this class 
				Coordsys_construct, 0,	// constructor fx and parameters
				Coordsys_props, Coordsys_methods, 
				NULL, NULL);
	return ret;
}


//==================================================================
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//
//   MATRIX class
//


////////////////////////////////////////////////////////////////////
//
//  UTILITIES FOR MATRIX 

void matrcol_to_newintarray (ChMatrix<>* ma, int* &marray, int &nrows)
{
	nrows = ma->GetRows();
	marray = (int*) calloc(sizeof(int), nrows); 
	for (int i=0; i<nrows; i++) 
		marray[i]=(int)ma->GetElement(i,0);
}
void intarray_to_matrcol (ChMatrix<>* ma, int* marray, int nrows)
{
	for (int i=0; i<nrows; i++) 
		ma->SetElement(i,0, (double)marray[i]);
}


////////////////////////////////////////////////////////////////////
//
// PROPERTIES
//

// ------- Properties list -------------------------------
//
static JSPropertySpec Matrix_props[] = {
    {"rows",          0,	JSPROP_ENUMERATE},
    {"columns",       1,	JSPROP_ENUMERATE},
    {0}
}; 

GET_JS_PARSE_BEGIN(Matrix_get, ChMatrix<>*)
	GET_JS_PROP (0,  chjs_from_int(cx,vp, this_data->GetRows()) )
	GET_JS_PROP (1,  chjs_from_int(cx,vp, this_data->GetColumns()) )
GET_JS_PARSE_END

SET_JS_PARSE_BEGIN(Matrix_set, ChMatrix<>*)
	SET_JS_PROP (0,	&chjs_int,	this_data->Reset(chjs_to_int(cx,vp),  this_data->GetColumns()) )
	SET_JS_PROP (1,	&chjs_int,	this_data->Reset(this_data->GetRows(),chjs_to_int(cx,vp)) )
SET_JS_PARSE_END



////////////////////////////////////////////////////////////////////
//
// METHODS
//

DEF_JS_BUILDER (Matrix_construct, ChMatrix<>*)
	if (argc==0)
		this_data = new ChMatrixDynamic<>();
	if (argc==1)
		this_data = new ChMatrixDynamic<>(chjs_to_int(cx, argv+0), 1);
	if (argc==2)
		this_data = new ChMatrixDynamic<>(chjs_to_int(cx, argv+0), chjs_to_int(cx, argv+1));
DEF_JS_BUILDEND

ChJS_FINALIZER (Matrix_finalize, ChMatrix<>*)


DEF_JS_FUNCTION(jsMaCreatexy, ChMatrix<>*, 2)
  PARCHK(0, &chjs_int)
  PARCHK(1, &chjs_int)
  this_data = new ChMatrixDynamic<>(chjs_to_int(cx, argv+0), chjs_to_int(cx, argv+1));
  chjs_from_dataNEW(cx,rval, this_data, &chjs_Matrix);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSetEl, ChMatrix<>*, 3)
  PARCHK(0, &chjs_int);
  PARCHK(1, &chjs_int);
  PARCHK(2, &chjs_double);
  int mi = chjs_to_int(cx, argv+0);
  int mj = chjs_to_int(cx, argv+1);
  if ((mi >= this_data->GetRows()) || (mj >= this_data->GetColumns()))
	{ JS_ReportError(cx,"You are setting an element out of matrix boundary"); return JS_FALSE;}
  this_data->SetElement(mi,mj,chjs_to_double(cx, argv+2));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaGetEl, ChMatrix<>*, 2)
  PARCHK(0, &chjs_int);
  PARCHK(1, &chjs_int);
  int mi = chjs_to_int(cx, argv+0);
  int mj = chjs_to_int(cx, argv+1);
  if ((mi >= this_data->GetRows()) || (mj >= this_data->GetColumns()))
	{ JS_ReportError(cx,"You are getting an element out of matrix boundary"); return JS_FALSE;}
  chjs_from_double(cx, rval, this_data->GetElement(mi,mj));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSafeSetEl, ChMatrix<>*, 3)
  PARCHK(0, &chjs_int);
  PARCHK(1, &chjs_int);
  PARCHK(2, &chjs_double);
  int mi = chjs_to_int(cx, argv+0);
  int mj = chjs_to_int(cx, argv+1);
  if ((mi >= this_data->GetRows()) || (mj >= this_data->GetColumns()))
  {
	  ChMatrixDynamic<> mtmp; mtmp.CopyFromMatrix(*this_data);
	  this_data->Reset(ChMax(mi+1, this_data->GetRows()), ChMax(mj+1, this_data->GetColumns()) );
	  this_data->PasteMatrix(&mtmp,0,0);
  }
  this_data->SetElement(mi,mj,chjs_to_double(cx, argv+2));
DEF_JS_FUNEND


DEF_JS_FUNCTION(jsMaCopy, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Matrix);
  this_data->CopyFromMatrix(*(ChMatrix<>*)chjs_to_data(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaCopyT, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Matrix);
  this_data->CopyFromMatrixT(*(ChMatrix<>*)chjs_to_data(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaReset, ChMatrix<>*, 2)
  PARCHK(0, &chjs_int);
  PARCHK(1, &chjs_int);
  this_data->Reset(chjs_to_int(cx, argv+0),chjs_to_int(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaFillElem, ChMatrix<>*, 1)
  PARCHK(0, &chjs_double);
  this_data->FillElem(chjs_to_double(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaFillDiag, ChMatrix<>*, 1)
  PARCHK(0, &chjs_double);
  this_data->FillDiag(chjs_to_double(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaFillRandom, ChMatrix<>*, 2)
  PARCHK(0, &chjs_double);
  PARCHK(1, &chjs_double);
  this_data->FillRandom(chjs_to_double(cx, argv+0), chjs_to_double(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaMultiply, ChMatrix<>*, 2)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  this_data->MatrMultiply(*(ChMatrix<>*)chjs_to_data(cx, argv+0), *(ChMatrix<>*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaMultiplyT, ChMatrix<>*, 2)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  this_data->MatrMultiplyT(*(ChMatrix<>*)chjs_to_data(cx, argv+0), *(ChMatrix<>*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaTMultiply, ChMatrix<>*, 2)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  this_data->MatrTMultiply(*(ChMatrix<>*)chjs_to_data(cx, argv+0), *(ChMatrix<>*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaAdd, ChMatrix<>*, 2)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  this_data->MatrAdd(*(ChMatrix<>*)chjs_to_data(cx, argv+0), *(ChMatrix<>*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSub, ChMatrix<>*, 2)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  this_data->MatrSub(*(ChMatrix<>*)chjs_to_data(cx, argv+0), *(ChMatrix<>*)chjs_to_data(cx, argv+1));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaInc, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Matrix);
  this_data->MatrInc(*(ChMatrix<>*)chjs_to_data(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaScale, ChMatrix<>*, 1)
  PARCHK(0, &chjs_double);
  this_data->MatrScale(chjs_to_double(cx, argv+0));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaTranspose, ChMatrix<>*, 0)
  this_data->MatrTranspose();
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaDot, ChMatrix<>*, 2)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  chjs_from_double(cx, rval, ChMatrix<>::MatrDot((ChMatrix<>*)chjs_to_data(cx, argv+0), (ChMatrix<>*)chjs_to_data(cx, argv+1))  );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaQtoA, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Quaternion);  
  ChMatrix33<> tmp;
  tmp.Set_A_quaternion(*(Quaternion*)chjs_to_data(cx, argv+0));
  this_data->CopyFromMatrix(tmp);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaAtoQ, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Quaternion); 
  if ((this_data->GetRows()!=3) || (this_data->GetColumns()!=3))
	{ JS_ReportError(cx,"AtoQ works only for 3x3 matrices"); return JS_FALSE;}
  ChMatrix33<> tmp; 
  tmp.CopyFromMatrix(*this_data);
  *(Quaternion*)chjs_to_data(cx, argv+0) = tmp.Get_A_quaternion();
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaVtoX, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Vector);  
  ChMatrix33<> tmp;
  tmp.Set_X_matrix(*(Vector*)chjs_to_data(cx, argv+0));
  this_data->CopyFromMatrix(tmp);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMa_x_Vect, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Vector); 
  if ((this_data->GetRows()!=3) || (this_data->GetColumns()!=3))
	{ JS_ReportError(cx,"ChMatrix<> * Vector works only for 3x3 matrices"); return JS_FALSE;}
  ChMatrix33<> tmp;
  tmp.CopyFromMatrix(*this_data);
  chjs_from_dataNEW(cx, rval, new Vector(tmp.Matr_x_Vect(*(Vector*)chjs_to_data(cx, argv+0)) ), &chjs_Vector) ;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaT_x_Vect, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Vector); 
  if ((this_data->GetRows()!=3) || (this_data->GetColumns()!=3))
	{ JS_ReportError(cx,"MatrixT * Vector works only for 3x3 matrices"); return JS_FALSE;}
  ChMatrix33<> tmp;
  tmp.CopyFromMatrix(*this_data);
  chjs_from_dataNEW(cx, rval, new Vector(tmp.MatrT_x_Vect(*(Vector*)chjs_to_data(cx, argv+0)) ), &chjs_Vector) ;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMa34_x_Quat, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Quaternion); 
  if ((this_data->GetRows()!=3) || (this_data->GetColumns()!=4))
	{ JS_ReportError(cx,"Matrix * Quaternion works only for 3x4 matrices"); return JS_FALSE;}
  chjs_from_dataNEW(cx, rval, new Vector(this_data->Matr34_x_Quat(*(Quaternion*)chjs_to_data(cx, argv+0)) ), &chjs_Vector) ;
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaDet, ChMatrix<>*, 0)  
  chjs_from_double(cx, rval, ChLinearAlgebra::Det(*this_data));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaInvert, ChMatrix<>*, 0)  
  ChLinearAlgebra::Invert(*this_data);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaCond, ChMatrix<>*, 0)  
  chjs_from_double(cx, rval, ChLinearAlgebra::ConditionNumber(*this_data));
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaRcond, ChMatrix<>*, 0)  
  chjs_from_double(cx, rval, 1/(ChLinearAlgebra::ConditionNumber(*this_data)) );
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSvd, ChMatrix<>*, 3)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  PARCHK(2, &chjs_Matrix);
  double cond;
  ChLinearAlgebra::SVD(*this_data, *(ChMatrix<>*)chjs_to_data(cx, argv+0), *(ChMatrix<>*)chjs_to_data(cx, argv+1), *(ChMatrix<>*)chjs_to_data(cx, argv+2), cond);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSolveTFQMR, ChMatrix<>*, 5)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  PARCHK(2, &chjs_Matrix);  
  PARCHK(3, &chjs_double);
  PARCHK(4, &chjs_int);
  ChMatrixDynamic<> m1; m1.CopyFromMatrix(*(ChMatrix<>*)chjs_to_data(cx, argv+1));
  ChMatrixDynamic<> m2; m2.CopyFromMatrix(*(ChMatrix<>*)chjs_to_data(cx, argv+2));
  chjs_from_int(cx, rval,
			ch_iterative_TFQMR_easy(
			*this_data, 
			m1,	m2,
			chjs_to_double(cx, argv+3),
			chjs_to_int(cx, argv+4)) );
DEF_JS_FUNEND


DEF_JS_FUNCTION(jsMaSolveLinSys, ChMatrix<>*, 3)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  PARCHK(2, &chjs_Matrix);
  double det;
  int* pivarray; int nrows;
  ChMatrix<>* mb = (ChMatrix<>*)chjs_to_data(cx, argv+0);
  ChMatrix<>* mx = (ChMatrix<>*)chjs_to_data(cx, argv+1);
  ChMatrix<>* mpi = (ChMatrix<>*)chjs_to_data(cx, argv+2);
  ChMatrixDynamic<>* mthis = new ChMatrixDynamic<>();
  mthis->CopyFromMatrix(*this_data);
  ChMatrixDynamic<>* mbback = new ChMatrixDynamic<>();
  mbback->CopyFromMatrix(*mb);
  mx->Reset(this_data->GetRows(),1);  // reset X sizes
  mpi->Reset(this_data->GetRows(),1);  // reset pivot sizes

  matrcol_to_newintarray(mpi, pivarray, nrows);
  ChLinearAlgebra::Solve_LinSys(*mthis, mbback, mx, pivarray, &det);
  intarray_to_matrcol(mpi, pivarray, nrows);

  free (pivarray);
  delete mthis;
  delete mbback;
  chjs_from_double(cx, rval, det);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaDecomposeLU, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Matrix);
  double det;
  int* pivarray; int nrows;
  ChMatrix<>* m0 = (ChMatrix<>*)chjs_to_data(cx, argv+0);
  m0->Reset(this_data->GetRows(),1);  // reset pivot size
  matrcol_to_newintarray(m0, pivarray, nrows);
  ChLinearAlgebra::Decompose_LU(*this_data, pivarray, &det);
  intarray_to_matrcol(m0, pivarray, nrows);
  free (pivarray);
  chjs_from_double(cx, rval, det);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSolveLU, ChMatrix<>*, 3)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  PARCHK(2, &chjs_Matrix);
  int* pivarray; int nrows;
  ChMatrix<>* m0 = (ChMatrix<>*)chjs_to_data(cx, argv+0);
  ChMatrix<>* m1 = (ChMatrix<>*)chjs_to_data(cx, argv+1);
  ChMatrix<>* m2 = (ChMatrix<>*)chjs_to_data(cx, argv+2);
  m1->Reset(this_data->GetRows(),1);  // reset X size
  matrcol_to_newintarray(m2, pivarray, nrows);
  ChLinearAlgebra::Solve_LU(*this_data, m0, m1, pivarray);
  intarray_to_matrcol(m2, pivarray, nrows);
  free (pivarray);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaDecomposeLDL, ChMatrix<>*, 1)
  PARCHK(0, &chjs_Matrix);
  double det;
  int* pivarray; int nrows;
  ChMatrix<>* m0 = (ChMatrix<>*)chjs_to_data(cx, argv+0);
  m0->Reset(this_data->GetRows(),1);  // reset pivot size
  matrcol_to_newintarray(m0, pivarray, nrows);
  ChLinearAlgebra::Decompose_LDL(*this_data,pivarray, &det);
  intarray_to_matrcol(m0, pivarray, nrows);
  free (pivarray);
  chjs_from_double(cx, rval, det);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSolveLDL, ChMatrix<>*, 3)
  PARCHK(0, &chjs_Matrix);
  PARCHK(1, &chjs_Matrix);
  PARCHK(2, &chjs_Matrix);
  int* pivarray; int nrows;
  ChMatrix<>* m0 = (ChMatrix<>*)chjs_to_data(cx, argv+0);
  ChMatrix<>* m1 = (ChMatrix<>*)chjs_to_data(cx, argv+1);
  ChMatrix<>* m2 = (ChMatrix<>*)chjs_to_data(cx, argv+2);
  m1->Reset(this_data->GetRows(),1);  // reset X size
  matrcol_to_newintarray(m2, pivarray, nrows);
  ChLinearAlgebra::Solve_LDL(*this_data, m0, m1, pivarray);
  intarray_to_matrcol(m2, pivarray, nrows);
  free (pivarray);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaPrint, ChMatrix<>*, 0)
for (int mi= 0; mi< this_data->GetRows(); mi++) {
	for (int mj= 0;  mj< this_data->GetColumns();   mj++)  {
		char num[30]; sprintf(num, "%g	", this_data->GetElement(mi,mj)); 
		CHGLOBALS_JS().chjsEngine->chjs_Print(num); }
	CHGLOBALS_JS().chjsEngine->chjs_Print("\n"); }
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaToString, ChMatrix<>*, 0)
static char mybuff[2000];
strcpy(mybuff,"");
sprintf(mybuff,"matrix, %d rows x %d columns ", this_data->GetRows(), this_data->GetColumns());
chjs_from_string(cx,rval, mybuff);
DEF_JS_FUNEND

DEF_JS_FUNCTION(jsMaSetIdentity, ChMatrix<>*, 0)  
  this_data->SetIdentity();
DEF_JS_FUNEND




// ------- Method list -------------------------------
//
static JSFunctionSpec Matrix_methods[] = {
    {"SetEl",			jsMaSetEl,			3},
    {"GetEl",			jsMaGetEl,			2},
	{"SafeSetEl",		jsMaSafeSetEl,		3},
    {"Copy",			jsMaCopy,			1},
    {"CopyT",			jsMaCopyT,			1},
    {"Reset",			jsMaReset,			2},
    {"FillElem",		jsMaFillElem,		1},
    {"FillDiag",		jsMaFillDiag,		1},
    {"FillRandom",		jsMaFillRandom,		2},
    {"Multiply",		jsMaMultiply,		2},
    {"MultiplyT",		jsMaMultiplyT,		2},
    {"TMultiply",		jsMaTMultiply,		2},
    {"Add",				jsMaAdd,			2},
    {"Sub",				jsMaSub,			2},
    {"Inc",				jsMaInc,			1},
    {"Scale",			jsMaScale,			1},
    {"Transpose",		jsMaTranspose,		0},
    {"Dot",				jsMaDot,			2},
    {"AtoQ",			jsMaAtoQ,			1},
    {"QtoA",			jsMaQtoA,			1},
    {"VtoX",			jsMaVtoX,			1},
    {"A_x_V",			jsMa_x_Vect,		1},
    {"At_x_V",			jsMaT_x_Vect,		1},
    {"G_x_Q",			jsMa34_x_Quat,		1},
    {"Det",				jsMaDet,			0},
    {"Cond",			jsMaCond,			0},
    {"Rcond",			jsMaRcond,			0},
    {"SVD",				jsMaSvd,			0},
    {"TFQMR",			jsMaSolveTFQMR,		5},
    {"Invert",			jsMaInvert,			0},
    {"SolveLinSys",		jsMaSolveLinSys,	0},
    {"DecomposeLU",		jsMaDecomposeLU,	0},
    {"SolveLU",			jsMaSolveLU,		0},
	{"DecomposeLDL",	jsMaDecomposeLDL,	0},
    {"SolveLDL",		jsMaSolveLDL,		0},
    {"Print",			jsMaPrint,			0},
    {"toString",		jsMaToString,		0},
	{"SetIdentity",		jsMaSetIdentity,	0},
	{0}
};

 
////////////////////////////////////////////////////////////////////
//
// CLASS DATA
//

// GLOBAL CLASS DATA (external)
JSClass chjs_Matrix= {
    "Matrix", JSCLASS_HAS_RESERVED_SLOTS(2),
    JS_PropertyStub,  JS_PropertyStub,  Matrix_get,		  Matrix_set,
    JS_EnumerateStub, JS_ResolveStub,   JS_ConvertStub,   Matrix_finalize,
};

JSClass* achjs_Matrix() {return &chjs_Matrix;}

   
////////////////////////////////////////////////////////////////////
//
// INITIALIZATION 
// 
JSObject* ChJS_InitClass_Matrix(JSContext* cx, JSObject* glob, JSObject* parent)
{
	JSObject* ret = JS_InitClass(cx, glob, 
				parent,					// parent prototype (parent class)
				&chjs_Matrix,			// this class 
				Matrix_construct, 0,	// constructor fx and parameters
				Matrix_props, Matrix_methods, 
				NULL, NULL);
	return ret;
}

} // END_OF_NAMESPACE____