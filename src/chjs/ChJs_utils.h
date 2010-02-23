#ifndef CHJSUTILS_H
#define CHJSUTILS_H

///////////////////////////////////////////////////
//
//   ChJs_utils.h
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
//   Utility functions for easy embedding of Chrono
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <math.h>
#include "jsapi.h"

#include "core/ChLists.h"
#include "physics/ChProplist.h"



namespace chrono
{

// 'fake' classes, for

extern JSClass chjs_double;
JSClass* achjs_double();  // get address of static 'Javascript class' structure
 
extern JSClass chjs_int;
JSClass* achjs_int();  // get address of static 'Javascript class' structure

extern JSClass chjs_string;
JSClass* achjs_string();  // get address of static 'Javascript class' structure

extern JSClass chjs_boolean;
JSClass* achjs_boolean();  // get address of static 'Javascript class' structure

extern JSClass chjs_object;
JSClass* achjs_object();  // get address of static 'Javascript class' structure

extern JSClass chjs_null;
JSClass* achjs_null();  // get address of static 'Javascript class' structure



// Initiates global 'utility' classes, such as the fake
// classes above. Must be called BEFORE all other
// chrono javascript classes are used. Returns TRUE  for Ok,.

int ChJS_InitClasses_Utils(JSContext* cx, JSObject* glob);


// JS --> C++
//
// Convert JS values to specific C++ data
// NOTE! JS value type must be already checked with previous
// function chjs_ValueIsObjecType() !!

int    chjs_to_int(JSContext* cx, jsval* vp);
double chjs_to_double(JSContext* cx, jsval* vp);
bool   chjs_to_bool(JSContext* cx, jsval* vp);
char*  chjs_to_string(JSContext* cx, jsval* vp);
void*  chjs_to_data(JSContext* cx, jsval* vp);


//  C++ -> JS
//
// .. and vice versa: convert specific C++ data to JS values:
// Note: no type checking is required here..

void chjs_from_int(JSContext* cx, jsval* vp, int mint);
void chjs_from_double(JSContext* cx, jsval* vp, double mdouble);
void chjs_from_bool(JSContext* cx, jsval* vp, bool mbool);
void chjs_from_string(JSContext* cx, jsval* vp, char* mstring);
void chjs_from_data(JSContext* cx, jsval* vp, void* mdata, JSClass* clasp);
void chjs_from_dataNEW(JSContext* cx, jsval* vp, void* mdata, JSClass* clasp);
		// ..dataNEW: as before, but also sets a ___own_ prop, to remember that (void*) mdata points
		// to something that javascript garbage collector must delete(), i.e. it is not application-private


// Check one value has correct type (for checking function parameters, etc.),
// where type class can include the fake classes chjs_double chjs_int etc.

JSBool chjs_ValueIsObjectType(JSContext* cx, jsval va, JSClass* clasp);



// Converts argument val to desired object type (class), if possible, and returns TRUE.
// But if the incoming object type was wrong, or not upcastable, returns FALSE (and null object)!

JSBool chjs_ValueToObjectType(JSContext* cx, jsval va, JSObject** obj, JSClass* clasp);



// Report JS error like "Type 'foo' of parameter 2 is wrong, it should be 'abcd' "

JSBool chjs_RepParErr(JSContext* cx, int npar, JSClass* clasp);


// Function to build a tree list of properties for objects

void chjs_obj_to_objtree(JSContext *cx, JSObject *lookobj, ChList<chjs_propdata>* mlist, int maxrec, int nrec);

void chjs_print_objtree(JSContext *cx, int tab, ChList<chjs_propdata>* mylist);


/////////////////////////////////////////////////////////////////////////////////////
///
/// MACROS FOR EASY WRAPPING
///

#define SET_JS_PROP(myid,myclasp,myfunct)   case myid: \
					{if (!chjs_ValueIsObjectType(cx,*vp, myclasp)) {chjs_RepParErr(cx,-1, myclasp); return JS_FALSE;}; \
					myfunct; \
					break; }

#define SET_JS_PARSE_BEGIN(setparser,mdatatypep) static JSBool setparser (JSContext *cx, JSObject *obj, jsval id, jsval *vp) \
					{jsval v; JS_GetReservedSlot(cx, obj, 0, &v); \
					mdatatypep this_data = (mdatatypep) JSVAL_TO_PRIVATE(v); \
					if (!this_data)	{JS_ReportError(cx,"ERROR getting null object\n"); return JS_FALSE;}\
					if (JSVAL_IS_INT(id)) { \
					switch (JSVAL_TO_INT(id)) {

#define SET_JS_PARSE_END	default: break; } } \
							return JS_TRUE; }


#define GET_JS_PROP(myid,myfunz)   case myid: {\
					myfunz; \
					break; }

#define GET_JS_PARSE_BEGIN(getparser,mdatatypep)  static JSBool getparser (JSContext *cx, JSObject *obj, jsval id, jsval *vp) { \
			jsval v; JS_GetReservedSlot(cx, obj, 0, &v); \
			mdatatypep this_data = (mdatatypep) JSVAL_TO_PRIVATE(v); \
			if (!this_data)	{ JS_ReportError(cx,"ERROR getting null object\n");	return JS_FALSE; } \
			if (JSVAL_IS_INT(id)) { \
			switch (JSVAL_TO_INT(id)) {

#define GET_JS_PARSE_END  default: break; } } \
			return JS_TRUE; }

#define DEF_JS_FUNCTION(funname,mdatatypep,numparams)  \
				static JSBool funname (JSContext *cx, JSObject *obj, uintN argc, jsval *argv, jsval *rval) { \
				 jsval v; JS_GetReservedSlot(cx, obj, 0, &v); \
				 mdatatypep this_data = (mdatatypep) JSVAL_TO_PRIVATE(v); \
				 *rval = JSVAL_VOID; \
				 if (argc != numparams) {JS_ReportError(cx, "Function needs %d parameters.", numparams); return JS_FALSE;}

#define DEF_JS_FUNCTION_NPARS(funname,mdatatypep)  \
				static JSBool funname (JSContext *cx, JSObject *obj, uintN argc, jsval *argv, jsval *rval) { \
				 jsval v; JS_GetReservedSlot(cx, obj, 0, &v); \
				 mdatatypep this_data = (mdatatypep) JSVAL_TO_PRIVATE(v); \
				 *rval = JSVAL_VOID;

#define PARCHK(numparam,myclasp) if (!chjs_ValueIsObjectType(cx,argv[numparam], myclasp)) {chjs_RepParErr(cx, 1 , myclasp ); return JS_FALSE;}

#define DEF_JS_FUNEND		return JS_TRUE;}

#define CHJS_FINALIZER(funname,mdatatypep)   JS_STATIC_DLL_CALLBACK(void) funname (JSContext *cx, JSObject *obj) { \
				jsval v; JS_GetReservedSlot(cx, obj, 1, &v); \
				if (v == JSVAL_TRUE) { \
					mdatatypep mdata; \
					JS_GetReservedSlot(cx, obj, 0, &v); \
					mdata = (mdatatypep) JSVAL_TO_PRIVATE(v);  \
					if (mdata) delete mdata;  \
				} }


#define DEF_JS_BUILDER(funname,mtypep)  \
			   static JSBool funname (JSContext *cx, JSObject *obj, uintN argc, jsval *argv, jsval *rval) { \
				mtypep this_data;\
				*rval = JSVAL_VOID;

#define DEF_JS_BUILDEND  	JS_SetReservedSlot(cx, obj, 0, PRIVATE_TO_JSVAL(this_data)); \
							JS_SetReservedSlot(cx, obj, 1, JSVAL_TRUE); \
							return JS_TRUE; }



} // END_OF_NAMESPACE____

#endif

