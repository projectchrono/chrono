///////////////////////////////////////////////////
//
//   ChFunction_Matlab.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Matlab.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Matlab> a_registration_matlab;

ChFunction_Matlab::ChFunction_Matlab ()
{
	strcpy(this->mat_command,"x*2+x^2");
}

void ChFunction_Matlab::Copy (ChFunction_Matlab* source)
{
	strcpy(this->mat_command, source->mat_command);
}

ChFunction* ChFunction_Matlab::new_Duplicate ()
{
	ChFunction_Matlab* m_func;
	m_func = new ChFunction_Matlab;
	m_func->Copy(this);
	return (m_func);
}

double ChFunction_Matlab::Get_y      (double x)
{
	double ret = 0;

	static char m_eval_command[CHF_MATLAB_STRING_LEN+20];

	#ifdef CH_MATLAB
	 // no function: shortcut!
	if (*this->mat_command == NULL) return 0.0;

	 // set string as "x=[x];ans=[mat_command]"
	 sprintf (m_eval_command, "x=%g;ans=%s;", x,this->mat_command);

	 // EVAL string, retrieving y = "ans"
	 ret = ChGLOBALS().Mat_Eng_Eval(m_eval_command);

	#else
	 ret = 0.0;
	#endif

	return ret;
}

void ChFunction_Matlab::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << mat_command;
}

void ChFunction_Matlab::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> mat_command;
}

void ChFunction_Matlab::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_MATLAB  \n";

	//***TO DO***
}






} // END_OF_NAMESPACE____


// eof
