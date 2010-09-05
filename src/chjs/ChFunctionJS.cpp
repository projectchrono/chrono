///////////////////////////////////////////////////
//
//   ChFunction.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <iostream>
#include <string.h>

#include "physics/ChGlobal.h"

#include "jsapi.h"
#include "unit_JS/ChJs_funct.h"
#include "unit_JS/ChJs_Engine.h"
#include "unit_JS/ChGlobalJS.h"
#include "unit_JS/ChFunctionJS.h"

namespace chrono
{




int ChFunctionOptvarToolsJS::OptVariablesToVector(ChFunction* funct, ChMatrix<>* mv, int offset)
{
	ChList<chjs_propdata>    mtree;
	ChList<chjs_fullnamevar> mlist;
	double mval=0;
	int    ind= 0;

	funct->MakeOptVariableTree(&mtree);
	funct->VariableTreeToFullNameVar(&mtree, &mlist);
	if (mv->GetColumns()!=1)
		mv->Reset(mlist.Count()+offset,1);

	CHGLOBALS_JS().chjsEngine->chjs_contextclass = chjs_cast_funct(funct);
	CHGLOBALS_JS().chjsEngine->chjs_contextdata  = funct;	// *** ...INTO CONTEXT

	for (ChNode<chjs_fullnamevar>* mnode = mlist.GetHead(); mnode; mnode=mnode->next)
	{
		if (CHGLOBALS_JS().chjsEngine->chjs_Eval(mnode->data->propname, &mval))
		{
			mv->SetElement(offset+ind, 0 , mval);
			ind++;
		}
	}

	CHGLOBALS_JS().chjsEngine->chjs_contextclass = NULL;
	CHGLOBALS_JS().chjsEngine->chjs_contextdata  = NULL;	// *** ...OUT FROM CONTEXT

	return ind;
}


int ChFunctionOptvarToolsJS::VectorToOptVariables(ChFunction* funct, ChMatrix<>* mv, int offset)
{
	ChList<chjs_propdata>    mtree;
	ChList<chjs_fullnamevar> mlist;
	double mval=0;
	int    ind= 0;
	jsval vp;

	funct->MakeOptVariableTree(&mtree);
	funct->VariableTreeToFullNameVar(&mtree, &mlist);

	CHGLOBALS_JS().chjsEngine->chjs_contextclass = chjs_cast_funct(funct);
	CHGLOBALS_JS().chjsEngine->chjs_contextdata  = funct;	// *** ...INTO CONTEXT

	for (ChNode<chjs_fullnamevar>* mnode = mlist.GetHead(); mnode; mnode=mnode->next)
	{
		jsdouble jsd = mv->GetElement(offset+ind,0);
		JS_NewDoubleValue(CHGLOBALS_JS().chjsEngine->cx, jsd, &vp);
		JS_SetProperty(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj, mnode->data->propname, &vp);
		ind++;
	}

	CHGLOBALS_JS().chjsEngine->chjs_contextclass = NULL;
	CHGLOBALS_JS().chjsEngine->chjs_contextdata  = NULL;	// *** ...OUT FROM CONTEXT

	return ind;
}




//////////////////////////////////////
//////////////////////////////////////
// CLASS ChFunction_Jscript

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Jscript> a_registration_jscript;


ChFunction_Jscript::ChFunction_Jscript ()
{
	strcpy(this->js_command,"0");
	this->js_script= 0;
	this->js_error = 0;
}

void ChFunction_Jscript::Copy (ChFunction_Jscript* source)
{
	this->Set_Command(source->Get_Command());
	//strcpy(this->js_command, source->js_command);
	//this->js_script= NULL;
	//this->js_error = source->js_error;
}

ChFunction* ChFunction_Jscript::new_Duplicate ()
{
	ChFunction_Jscript* m_func;
	m_func = new ChFunction_Jscript;
	m_func->Copy(this);
	return (m_func);
}

void ChFunction_Jscript::Set_Command  (char* m_command)
{
	strcpy (this->js_command, m_command);

	// Just to have the evaluation with error reporter...
	double ret; int ok;
	ok = CHGLOBALS_JS().chjsEngine->chjs_Eval(this->js_command, &ret);

	if (!ok) 	this->js_error = TRUE;
	else 		this->js_error = FALSE;
}

void ChFunction_Jscript::Set_Variable (char* variablename, double value)
{
	jsdouble jsd = value;
	jsval vp;
	JS_NewDoubleValue(CHGLOBALS_JS().chjsEngine->cx, jsd, &vp);
	JS_SetProperty(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj, variablename, &vp);
}


double ChFunction_Jscript::Get_y      (double x)
{
	double ret = 0;

	// no function: shortcut!
	if (*this->js_command == 0) return 0.0;	// <<<<<

	// set the x value for the function
	Set_Variable("x", x);

	// The standard evaluation of this object won't report errors!
	// (error reporter will work only if the user does ::Set_Command() )
	int m_old_repmode = CHGLOBALS_JS().chjsEngine->chjs_reporter_mode;
	//if (this->js_error)
	CHGLOBALS_JS().chjsEngine->chjs_SetReporterMode(0); // NO REPORT!

	// >>>> EVALUATION
	// if not yet compiled, compile it!
	//
	int ok = CHGLOBALS_JS().chjsEngine->chjs_Eval(this->js_command, &ret);

	if (!ok) 	this->js_error = TRUE;
	else 		this->js_error = FALSE;

	//if (this->js_error)
	CHGLOBALS_JS().chjsEngine->chjs_SetReporterMode(m_old_repmode); // Restore rerror reporter

	return ret;
}

void ChFunction_Jscript::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << js_command;
}

void ChFunction_Jscript::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> js_command;
}

void ChFunction_Jscript::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_JSCRIPT  \n";

	//***TO DO***
}







} // END_OF_NAMESPACE____


// eof
