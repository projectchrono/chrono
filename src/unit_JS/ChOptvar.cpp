//////////////////////////////////////////////////
//
//   ChOptvar.h
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "physics/ChSystem.h"
#include "unit_JS/ChOptvar.h"
#include "unit_JS/ChJs_Engine.h"
#include "unit_JS/ChGlobalJS.h"

namespace chrono
{

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR OPTIMIZATION VARS


ChOptVar::ChOptVar ()
{
	strcpy (varname, "");
	varnum = 0;
	lock = FALSE;
	lim_sup = +1000;
	lim_inf = -1000;
	compiled = FALSE;
}

ChOptVar::~ChOptVar ()
{
	lock = FALSE; // test
}

void ChOptVar::Copy(ChOptVar* source)
{
		// first copy the parent class data...
	ChObj::Copy(source);
	strcpy (varname, source->varname);
	vartag = source->vartag;
	varnum = source->varnum;
	lock = source->lock;
	lim_sup = source->lim_sup;
	lim_inf = source->lim_inf;
	compiled = source->compiled;
}


//
//

void  ChOptVar::SetVarName (char myname[])
{
	strcpy (varname, myname);
	compiled = FALSE;

	double mvalue;
	if (!CHGLOBALS_JS().chjsEngine->chjs_Eval(this->GetVarName(),&mvalue))
	{
		jsdouble jsd = 0.0;
		jsval vp;
		JS_NewDoubleValue(CHGLOBALS_JS().chjsEngine->cx, jsd, &vp);
		JS_SetProperty(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj, this->GetVarName(), &vp);
	}
}

//
// Variable set/fetch
//

				// This function returns the value of the variable
				// The parameter 'database' points to the PSystem data
double ChOptVar::GetVarValue(void* database)
{
	double mvalue = 0;
	if (!compiled)
	{
		CHGLOBALS_JS().chjsEngine->chjs_Eval(this->GetVarName(),&mvalue);
	}
	else
	{
		CHGLOBALS_JS().chjsEngine->chjs_Eval(this->GetVarName(),&mvalue);		//**TO DO** use compiled vars
	}
	return mvalue;
}

				// This function sets the value of the variable
int ChOptVar::SetVarValue(double mval, void* database)
{
	jsdouble jsd = mval;
	jsval vp;
	JS_NewDoubleValue(CHGLOBALS_JS().chjsEngine->cx, jsd, &vp);

	JS_SetProperty(CHGLOBALS_JS().chjsEngine->cx, CHGLOBALS_JS().chjsEngine->jglobalObj, this->GetVarName(), &vp);

	return TRUE;
}


// Check if a variable is of type DOUBLE, and can be set and get
int ChOptVar::IsValid(void* database)
{
	double mvalue;
	if (CHGLOBALS_JS().chjsEngine->chjs_Eval(this->GetVarName(),&mvalue))
		return TRUE;
	else
		return FALSE;
}


int ChOptVar::Compile(void* database)
{
	// ***TO DO*** compile variable if possible
	return TRUE;
}


//////// FILE I/O

void ChOptVar::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChObj::StreamOUT(mstream);

		// stream out all member data
	mstream << varname;
	mstream << varnum;
	mstream << lock;
	mstream << lim_sup;
	mstream << lim_inf;
}

void ChOptVar::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChObj::StreamIN(mstream);

	mstream >> varname;
	mstream >> varnum;
	mstream >> lock;
	mstream >> lim_sup;
	mstream >> lim_inf;
	compiled = FALSE;

}

void ChOptVar::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "VARIABLE n." << varnum << " name=" << varname << " \n";
	// incomplete?
}




} // END_OF_NAMESPACE____


/////////////////////
