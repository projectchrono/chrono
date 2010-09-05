///////////////////////////////////////////////////
//
//   ChJs_Engine.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "core/ChLog.h"
#include "core/ChMathematics.h"
#include "ChJs_Engine.h"
#include "ChGlobalJS.h"
#include "physics/ChGlobal.h"


namespace chrono
{


	void ChJavascriptScript::DestroyJSScript() 
		{
			if (jscript) 
			{
				JS_DestroyScript(this->jengine->cx, this->jscript);
				this->jengine = 0; this->jscript = 0;
			}
		};



// The class members


ChJavascriptEngine::ChJavascriptEngine()
{
	  rt = NULL;
      cx = NULL;
	  jglobalObj= NULL;
	  chjs_contextclass = NULL;
	  chjs_contextdata =  NULL;
	  Init_Js_Eng();	// Creates Javascript context
}



ChJavascriptEngine::~ChJavascriptEngine()
{
	JS_DestroyContext(cx);
	JS_DestroyRuntime(rt);
};




/////////////////////////////////////////////////////////
//  JAVASCRIPT SPECIFIC
//


										// template for global class
static JSClass  __globalClass  = {
    "global",0,
    JS_PropertyStub,JS_PropertyStub,JS_PropertyStub,JS_PropertyStub,
    JS_EnumerateStub,JS_ResolveStub,JS_ConvertStub,JS_FinalizeStub
};




void ChJavascriptEngine::chjs_SetReporterMode(int mmode)
{
	if (this->chjs_reporter_mode != mmode)
	{
		this->chjs_reporter_mode = mmode;
		if (mmode == 0) JS_SetErrorReporter(cx, chjs_errorReporterVoid);
		if (mmode == 1) JS_SetErrorReporter(cx, chjs_errorReporter);
	}
}




int ChJavascriptEngine::Init_Js_Eng()
{
	chjs_reporter_mode = 1;

    rt = JS_NewRuntime(1000000);
    if (!rt)
	{
		GetLog()  - ChLog::CHERROR << "Could not start Javascript runtime!";
	}
    cx = JS_NewContext(rt, 8192);
    if (!cx)
	{
        GetLog()  - ChLog::CHERROR << "Can't create JavaScript context!";
	}

	JS_SetErrorReporter(cx, chjs_errorReporter);

	jglobalClass = __globalClass;
    jglobalObj = JS_NewObject(cx, &jglobalClass, 0, 0);

	////// Init some standard classes
	JS_InitStandardClasses(cx, jglobalObj);

	////// Init some global stuff
	chjs_SetReporterMode(0);
	jsval rval;
	JSScript* hea = JS_CompileFile(this->cx, this->jglobalObj, "chrono/javascript/chrono_startup.js");
	if (hea)
	{
		JSBool ok = JS_ExecuteScript(this->cx, this->jglobalObj, hea, &rval);
		JS_DestroyScript(cx, hea);
		if (!ok)
		{
			GetLog()  - ChLog::CHWARNING << "Could not load chrono/javascript/chrono_startup.js  header, useful for Chrono scripting";
		}
	}
	chjs_SetReporterMode(1);

	if (cx && rt && jglobalObj) return TRUE;
	return FALSE;
}




int ChJavascriptEngine::chjs_Eval(char* msource, double* mval)
{
	jsval rval;
    JSBool ok = 0;

    ok = JS_EvaluateScript(this->cx, this->jglobalObj, msource, strlen(msource),
                           0 , 0 , &rval);

    if (ok) {
        ok = JS_ValueToNumber(cx, rval, mval);
		if (!JSVAL_IS_NUMBER(rval))
		{
			ok = FALSE;
			*mval =  0;
		}
    }

	if (!ok)
	{
		*mval = 0.0; // default for error
		return FALSE;
	}
	return TRUE;
}




int ChJavascriptEngine::chjs_Eval(char* msource, char* result)
{
	jsval rval;
    JSBool ok = 0;

    ok = JS_EvaluateScript(this->cx, this->jglobalObj, msource, strlen(msource),
		0 , 0 , &rval);

	if (result)
	{
		if (ok)
		{
			if JSVAL_IS_VOID(rval) {strcpy(result,""); return TRUE;}
			JSString* str;
			str=JS_ValueToString(cx, rval);
			char* garb_string= JS_GetStringBytes(str);
			strcpy(result, garb_string);
			return TRUE;
		}
		else
		{
			strcpy(result, "");
			return FALSE;
		}
	}
	else return ok;

	return TRUE;
}



int ChJavascriptEngine::chjs_SetVar(char* mvarname, double mval)
{
	jsdouble jsd = mval;
	jsval vp;
	JS_NewDoubleValue(this->cx, jsd, &vp);
	return JS_SetProperty(this->cx, this->jglobalObj, mvarname, &vp);
}



int ChJavascriptEngine::chjs_Print(char* msource)
{
	CHGLOBALS_JS().GetScriptingLog() << msource;

	return TRUE;
}



int ChJavascriptEngine::chjs_FileToScript(JSScript** script, char* file)
{
	if (!(*script == NULL)) {
		JS_DestroyScript(this->cx, *script);
		*script = NULL;
	}
	if (file == NULL) return TRUE;
	if (*file == 0) return TRUE; // null script for "" file (empty string dialog)
	*script = JS_CompileFile(this->cx, this->jglobalObj, file);
	if (!(*script)) return FALSE;
	return TRUE;
}

int ChJavascriptEngine::FileToScript(ChScript& script, char* file)
{
	if (ChJavascriptScript* mjs = dynamic_cast<ChJavascriptScript*> (&script))
	{
		if (mjs->jscript)	
			JS_DestroyScript(mjs->jengine->cx, mjs->jscript);
		mjs->jengine = this;
		return chjs_FileToScript(&mjs->jscript, file);
	}
	return false;
}

int ChJavascriptEngine::ExecuteScript(ChScript& script) 
{
	if (ChJavascriptScript* mjs = dynamic_cast<ChJavascriptScript*> (&script))
	{
		if (!mjs->jscript) return 0;
		jsval jsresult;
		return JS_ExecuteScript(this->cx, this->jglobalObj,
			mjs->jscript, &jsresult);
	}
	return 0;
};


						/* custom Javascript error reporter */
extern void chjs_errorReporter(JSContext *cx, const char *message, JSErrorReport *report)
{
	static char merbuf[350];

    if(report == NULL)	{
		sprintf((char*)merbuf,"Javascript error! \njssh: %s\n", message);
	}
    else	{
		sprintf((char*)merbuf,"Javascript error! \n\n %s \n line: %i \n error: %s \n",
				report->filename, report->lineno, message);
	}
	CHGLOBALS_JS().chjsEngine->chjs_Print(merbuf);
}


						/* void Javascript error reporter */
extern void chjs_errorReporterVoid(JSContext *cx, const char *message, JSErrorReport *report)
{
 // do nothing
}





} // END_OF_NAMESPACE____



// End
