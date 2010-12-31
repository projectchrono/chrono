
///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoContactN.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoGPUcontN.h" 


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoGPUcontN> a_registration_ChLcpConstraintTwoGPUcontN;

  


void ChLcpConstraintTwoGPUcontN::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwo::StreamOUT(mstream);

		// stream out all member data..
	mstream << friction;

}

void ChLcpConstraintTwoGPUcontN::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwo::StreamIN(mstream);

		// stream in all member data..
	mstream >> friction;
}




} // END_OF_NAMESPACE____
