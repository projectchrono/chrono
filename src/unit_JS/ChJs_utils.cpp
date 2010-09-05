///////////////////////////////////////////////////
//
//   ChJs_utils.cpp
//
//	 CHRONO
//   ------
//   Multibody dinamics engine
//
//   Embedding utilities
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChJs_utils.h"
#include "ChJs_Engine.h"
#include "ChGlobalJS.h"
#include "physics/ChGlobal.h"

#include "jsdbgapi.h"
#include <errno.h>



namespace chrono
{



// FAKE CLASSES, FOR PARAMETER CHECK

JSClass chjs_double = {
    "double", 0,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
JSClass* achjs_double() {return &chjs_double;}

JSClass chjs_int = {
    "int", 0,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
JSClass* achjs_int() {return &chjs_int;}

JSClass chjs_string = {
    "string", 0,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
JSClass* achjs_string() {return &chjs_string;}

JSClass chjs_boolean = {
    "boolean", 0,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
JSClass* achjs_boolean() {return &chjs_boolean;}

JSClass chjs_null = {
    "null", 0,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
JSClass* achjs_null() {return &chjs_null;}

JSClass chjs_object = {
    "object", 0,
    NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL
};
JSClass* achjs_object() {return &chjs_object;}


static JSBool js_print(JSContext *cx, JSObject *jso, uintN argc, jsval *argv, jsval *rval)
{
     unsigned int i;
     JSString *str;

     //R3OBJ *obj = JS_GetContextPrivate(cx);

     for(i = 0; i < argc; i++)
         if(str = JS_ValueToString(cx, argv[i]))
			CHGLOBALS_JS().chjsEngine->chjs_Print(JS_GetStringBytes(str));

     return JS_TRUE;
}

static JSBool js_load(JSContext *cx, JSObject *obj, uintN argc, jsval *argv, jsval *rval)
{
    uintN i;
    JSString *str;
    const char *filename;
    JSScript *script;
    JSBool ok;
    jsval result;
    //JSErrorReporter older;

    for (i = 0; i < argc; i++) {
	str = JS_ValueToString(cx, argv[i]);
	if (!str)
	    return JS_FALSE;
	argv[i] = STRING_TO_JSVAL(str);
	filename = JS_GetStringBytes(str);
	errno = 0;
    //    older = JS_SetErrorReporter(cx, my_LoadErrorReporter);
	script = JS_CompileFile(cx, obj, filename);
	if (!script)
            ok = JS_FALSE;
	else {
            ok = JS_ExecuteScript(cx, obj, script, &result);
	    JS_DestroyScript(cx, script);
        }
    //    JS_SetErrorReporter(cx, older);
	if (!ok)
	    return JS_FALSE;
    }
    return JS_TRUE;
}

void chjs_obj_to_objtree(JSContext *cx, JSObject *lookobj, ChList<chjs_propdata>* mlist, int maxrec, int nrec)
{
	JSObject* origobj = lookobj;
	JSObject* childobj;
	while (lookobj)
	{
		JSIdArray* idar = JS_Enumerate(cx, lookobj);
		for (int i = 0; i<idar->length; i++)
		{
			jsid mid = *(idar->vector+i); 
			jsval mva;
			JS_IdToValue(cx, mid, &mva);
			//if (JSVAL_IS_INT(mva)) ...
			if (JSVAL_IS_STRING(mva))
			{
				char* mstring = NULL;
				mstring = JS_GetStringBytes(JS_ValueToString(cx, mva));
				chjs_propdata* mdata = new chjs_propdata;
				strcpy(mdata->propname, mstring);
				strcpy(mdata->label,    mstring);
				mdata->haschildren = FALSE;
				mlist->AddTail(mdata);

				// add also child, recursively
				jsval subva;
				JS_GetProperty(cx, origobj, mstring, &subva);
				if (JSVAL_IS_OBJECT(subva))
				{
					childobj = JSVAL_TO_OBJECT(subva);
					if (childobj)
					{
						mdata->haschildren = TRUE;
						if (nrec < maxrec)
							chjs_obj_to_objtree(cx, childobj, &mdata->children, maxrec, nrec+1);
					}
				}
			}
		}
		lookobj = JS_GetPrototype(cx,lookobj);  // continue scanning also parent proto..
	}
}

void chjs_print_objtree(JSContext *cx, int tab, ChList<chjs_propdata>* mylist)
{
	for (ChNode<chjs_propdata>* mnode = mylist->GetHead(); mnode != NULL; mnode= mnode->next)
	{
		for (int i=0; i<tab; i++)
			CHGLOBALS_JS().chjsEngine->chjs_Print("  ");
		CHGLOBALS_JS().chjsEngine->chjs_Print(mnode->data->propname);
		CHGLOBALS_JS().chjsEngine->chjs_Print("\n");
		if (mnode->data->children.GetHead())
			chjs_print_objtree(cx, tab+1, &mnode->data->children);
	}
}


static JSBool js_objtree(JSContext *cx, JSObject *obj, uintN argc, jsval *argv, jsval *rval)
{
	JSObject* lookobj;

	if (argc == 1)
		lookobj = JSVAL_TO_OBJECT(argv[0]);
	if (argc == 0)
	    lookobj = CHGLOBALS_JS().chjsEngine->jglobalObj;
	if (argc > 1)
	    return JS_FALSE;

	ChList<chjs_propdata> mylist;
	chjs_obj_to_objtree(cx, lookobj, &mylist, 1,0);
	chjs_print_objtree(cx, 0, &mylist);

	mylist.KillAll();

    return JS_TRUE;
}

static JSBool js_context(JSContext *cx, JSObject *jso, uintN argc, jsval *argv, jsval *rval)
{
     if ((CHGLOBALS_JS().chjsEngine->chjs_contextclass == NULL) || (CHGLOBALS_JS().chjsEngine->chjs_contextdata == NULL))
		*rval = OBJECT_TO_JSVAL(CHGLOBALS_JS().chjsEngine->jglobalObj);  // no context, return global
	 else
		chjs_from_data(cx, rval, CHGLOBALS_JS().chjsEngine->chjs_contextdata, CHGLOBALS_JS().chjsEngine->chjs_contextclass);
     return JS_TRUE;
}

static JSBool js_globals(JSContext *cx, JSObject *jso, uintN argc, jsval *argv, jsval *rval)
{
	*rval = OBJECT_TO_JSVAL(CHGLOBALS_JS().chjsEngine->jglobalObj);  // no context, return global
	return JS_TRUE;
}


static JSBool js_is_owned(JSContext *cx, JSObject *obj, uintN argc, jsval *argv, jsval *rval)
{
	JSObject* lookobj;
	if (argc == 1)
		lookobj = JSVAL_TO_OBJECT(argv[0]);
	else return JS_TRUE;
	jsval v;
	JS_GetReservedSlot(cx, lookobj, 1, &v);
    bool mdata = (bool)JSVAL_TO_BOOLEAN(v);
	chjs_from_bool(cx, rval, mdata);
	return JS_TRUE;
}

static JSBool js_set_owned(JSContext *cx, JSObject *obj, uintN argc, jsval *argv, jsval *rval)
{
	JSObject* lookobj;
	int mowned;
	if (argc == 2)
	{
		lookobj = JSVAL_TO_OBJECT(argv[0]);
		mowned = JSVAL_TO_BOOLEAN(argv[1]);
	}
	else return JS_TRUE;
	if (mowned)
		JS_SetReservedSlot(cx, lookobj, 1, JSVAL_TRUE);
	else
		JS_SetReservedSlot(cx, lookobj, 1, JSVAL_FALSE);
	return JS_TRUE;
}



//
// Initialization of globals
//

static JSFunctionSpec utils_methods[] = {
    {"print",            js_print,       0},
    {"load",             js_load,        1},
    {"objtree",          js_objtree,     1},
	{"context",			 js_context,     0},
	{"globals",			 js_globals,     0},
	{"is_jsowned",		 js_is_owned,    0},
	{"set_jsowned",		 js_set_owned,   0},
    {0}
};

int ChJS_InitClasses_Utils(JSContext* cx, JSObject* glob)
{
	// Define some global "utility" functions
	JS_DefineFunctions(cx, glob, utils_methods);

	// nothing special to do
	return TRUE;
}


// JS --> C++
//
// Convert JS values to specific C++ data
// NOTE! JS value type must be already checked with previous
// function chjs_ValueIsObjecType() !!

int chjs_to_int(JSContext* cx, jsval* vp)
{
	int32 mint;
	JS_ValueToInt32(cx, *vp, (int32*)&mint);
	return mint;
}
double chjs_to_double(JSContext* cx, jsval* vp)
{
	double mdouble;
	JS_ValueToNumber(cx, *vp, &mdouble);
	return mdouble;
}
bool chjs_to_bool(JSContext* cx, jsval* vp)
{
	bool mbool;
	JS_ValueToBoolean(cx, *vp, (int*)&mbool);
	return mbool;
}
char* chjs_to_string(JSContext* cx, jsval* vp)
{
	char* mstring = NULL;
	mstring = JS_GetStringBytes(JS_ValueToString(cx, *vp));
	return mstring;
}
void* chjs_to_data(JSContext* cx, jsval* vp)
{
	void* mdata;
	JSObject* mobj;
	mobj = JSVAL_TO_OBJECT(*vp);
	jsval v;
	JS_GetReservedSlot(cx, mobj, 0, &v);
    mdata = JSVAL_TO_PRIVATE(v);
	return mdata;
}


//  C++ -> JS
//
// .. and vice versa: convert specific C++ data to JS values:
// Note: no type checking is required here..

void chjs_from_int(JSContext* cx, jsval* vp, int mint)
{
	*vp = INT_TO_JSVAL(mint);
}
void chjs_from_double(JSContext* cx, jsval* vp, double mdouble)
{
	JS_NewDoubleValue(cx, mdouble, vp);
}
void chjs_from_bool(JSContext* cx, jsval* vp, bool mbool)
{
	*vp = BOOLEAN_TO_JSVAL(mbool);
}
void chjs_from_string(JSContext* cx, jsval* vp, char* mstring)
{
	JSString *jstr = JS_NewString(cx, mstring, (size_t) strlen(mstring));
	*vp = STRING_TO_JSVAL(jstr);
}
void chjs_from_data(JSContext* cx, jsval* vp, void* mdata, JSClass* clasp)
{
	JSObject *jso = JS_NewObject(cx, clasp, NULL, NULL);
    JS_SetReservedSlot(cx, jso, 0, PRIVATE_TO_JSVAL(mdata));
    JS_SetReservedSlot(cx, jso, 1, JSVAL_FALSE); // mdata belongs to application (never delete in GC!)
	*vp = OBJECT_TO_JSVAL(jso);
}
void chjs_from_dataNEW(JSContext* cx, jsval* vp, void* mdata, JSClass* clasp)
{
	JSObject *jso = JS_NewObject(cx, clasp, NULL, NULL);
    JS_SetReservedSlot(cx, jso, 0, PRIVATE_TO_JSVAL(mdata));
    JS_SetReservedSlot(cx, jso, 1, JSVAL_TRUE); // mdata is instatiated by JS context, can be deleted in GC
	*vp = OBJECT_TO_JSVAL(jso);
}

// Check one value has correct type (for checking function parameters, etc.),
// where type class can include the fake classes chjs_double chjs_int etc.

JSBool chjs_ValueIsObjectType(JSContext* cx, jsval va, JSClass* clasp)
{
	static JSObject* mproto;
	static JSObject* obj;

	// test for fake classes, for common types...

	if (clasp==&chjs_double)
		if (JSVAL_IS_NUMBER(va)) return JS_TRUE; else return JS_FALSE;

	if (clasp==&chjs_int)
		if (JSVAL_IS_INT(va)) return JS_TRUE; else return JS_FALSE;

	if (clasp==&chjs_boolean)
		if (JSVAL_IS_BOOLEAN(va)) return JS_TRUE; else return JS_FALSE;

	if (clasp==&chjs_null)
		if (JSVAL_IS_NULL(va)) return JS_TRUE; else return JS_FALSE;

	if (clasp==&chjs_string)
		if (JSVAL_IS_STRING(va)) return JS_TRUE; else return JS_FALSE;

	// test all other object types...

	obj= NULL;
	if (JSVAL_IS_OBJECT(va))
	{
	  obj = JSVAL_TO_OBJECT(va);
	  if (JS_InstanceOf(cx, obj, clasp, NULL))
		return JS_TRUE;
	  mproto = JS_GetPrototype(cx, obj);

	  // test also against parent class -maybe obj belongs to inherited class-
	  while (mproto)
	  {
		if (JS_InstanceOf(cx, mproto, clasp, NULL))
			return JS_TRUE;
		mproto = JS_GetPrototype(cx, mproto);
	  }
	  return JS_FALSE; // all checks failed...
	}
	else return JS_FALSE;

	return JS_FALSE;
}



// Converts argument val to desired object type (class), if possible, and returns TRUE.
// But if the incoming object type was wrong, or not upcastable, returns FALSE (and null object)!
JSBool chjs_ValueToObjectType(JSContext* cx, jsval va, JSObject** obj, JSClass* clasp)
{
	JSObject* mproto;
	*obj= NULL;
	if (JSVAL_IS_OBJECT(va))
	{
	  *obj = JSVAL_TO_OBJECT(va);
	  if (JS_InstanceOf(cx, *obj, clasp, NULL))
		return JS_TRUE;
	  mproto = JS_GetPrototype(cx, *obj);
	  // test also against parent class...
	  while (mproto)
	  {
		if (JS_InstanceOf(cx, mproto, clasp, NULL))
			return JS_TRUE;
		mproto = JS_GetPrototype(cx, mproto);
	  }
	  return JS_FALSE; // all checks failed...
	}
	else return JS_FALSE;

	return JS_TRUE;
}

JSBool chjs_RepParErr(JSContext* cx, int npar, JSClass* clasp)
{

	if (npar == -1)
		JS_ReportError(cx,"Parameter is not of correct type (should be of '%s' class)",clasp->name);
	else
		JS_ReportError(cx,"Parameter %d is not of correct type (should be of '%s' class)",npar, clasp->name);
	return JS_TRUE;
}



} // END_OF_NAMESPACE____

