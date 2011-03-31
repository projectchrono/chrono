///////////////////////////////////////////////////
//
//   ChFunction_Const.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChFunction_Const.h"


namespace chrono
{



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChFunction_Const> a_registration_const;

void ChFunction_Const::Copy (ChFunction_Const* source)
{
	Set_yconst  (source->C);
}

ChFunction* ChFunction_Const::new_Duplicate ()
{
	ChFunction_Const* m_func;
	m_func = new ChFunction_Const;
	m_func->Copy(this);
	return (m_func);
}


void ChFunction_Const::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);
		// serialize parent class too
	ChFunction::StreamOUT(mstream);

		// stream out all member data
	mstream << C;
}

void ChFunction_Const::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();
		// deserialize parent class too
	ChFunction::StreamIN(mstream);

		// stream in all member data
	mstream >> C;
}

void ChFunction_Const::StreamOUT(ChStreamOutAscii& mstream)
{
	mstream << "FUNCT_CONST  \n";

	//***TO DO***
}







} // END_OF_NAMESPACE____


// eof
