#ifndef CHOPTVAR_H
#define CHOPTVAR_H

//////////////////////////////////////////////////
//   
//   ChOptvar.h
//   
//   Optimize variable structure  -definition.
//   (used in Chrono to represent variables which
//    can be modified by optimization engine when
//    using the ChFxJavascript).
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <memory.h>

#include "core/ChMath.h"
#include "physics/ChObject.h"
#include "physics/ChGlobal.h"
#include "physics/ChFormule.h"
#include "unit_JS/ChApiJS.h"


namespace chrono 
{



#define CHCLASS_OPTVAR	23

/////////////////////////////////////
// CLASS FOR OPTIMIZATION VARIABLES 
//
/// Class for variables addressing properties of scriptable 
/// objects which can be subject to optimization.

class ChApiJS ChOptVar : public ChObj {

private:
	char varname[100];	// the variable 'string identifier'
	ChTag vartag;		// tag identifier for faster access to variable evaluation
	int varnum;			// number of variable in optimization vector
	int lock;			// the variable are disabled if lock = true
	double lim_sup;		// upper limit
	double lim_inf;		// lower limit
	int compiled;		// for internal use

public:
	ChOptVar ();	// builders and destroyers

	~ChOptVar ();
	void Copy(ChOptVar* source);

				/// This function returns the value of the variable
				/// The parameter 'database' points to the PSystem data
	double GetVarValue(void* database);

				/// This function sets the value of the variable 
	int SetVarValue(double mval, void* database);

				/// Returns FALSE if the string cannot not be parsed
				/// in the object   ChObj* database, and TRUE if it is parseable (i.e
				/// of type CHCLASS_FLOAT, i.e. "double" data, and both writeable/settable
	int IsValid(void* database);

				/// Compile the variable. i.e. the string is translated in the
				/// equivalenent Ch_Tag structure, and subsequent calls for Get/SetVarValue()
				/// will be much faster. Note: returns FALSE if translation unsuccesfull.
	int Compile(void* database);

				/// params, tags, flags, etc.
	void  SetVarName (char myname[]);
	char* GetVarName () {return varname;};

	void SetVarnum (signed int myvarnum) {varnum = myvarnum;}
	int  GetVarnum () {return varnum;}

	void SetLock (int m_lock) {lock= m_lock;}
	int  GetLock () {return lock;}

	void   SetLimSup (double msup) {lim_sup = msup;};
	double GetLimSup () {return lim_sup;};
	void   SetLimInf (double minf) {lim_inf = minf;};
	double GetLimInf () {return lim_inf;};

				// file i/o

					/// Method to allow deserializing a persistent binary archive (ex: a file)
					/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

					/// Method to allow serializing transient data into a persistent
					/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);

					/// Method to allow serialization of transient data in ascii,
					/// as a readable item, for example   "chrono::GetLog() << myobject;"
	void StreamOUT(ChStreamOutAscii& mstream);
};


} // END_OF_NAMESPACE____

#endif
