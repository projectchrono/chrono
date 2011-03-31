///////////////////////////////////////////////////
//
//   ChFunction_Ramp.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Ramp.h"


namespace chrono
{


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Ramp> a_registration_ramp;

void ChFunction_Ramp::Copy (ChFunction_Ramp* source)
{
	Set_y0  (source->y0);
	Set_ang (source->ang);
}

ChFunction* ChFunction_Ramp::new_Duplicate ()
{
	ChFunction_Ramp* m_func;
	m_func = new ChFunction_Ramp;
	m_func->Copy(this);
	return (m_func);
}


void ChFunction_Ramp::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << y0;
	mstream << ang;
}

void ChFunction_Ramp::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> y0;
	mstream >> ang;
}

void ChFunction_Ramp::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_RAMP  \n";

	//***TO DO***
}





} // END_OF_NAMESPACE____


// eof
