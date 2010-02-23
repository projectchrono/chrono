///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoFriction.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoFriction.h"


namespace chrono
{

// Register into the object factory, to enable run-time 
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoFriction> a_registration_ChLcpConstraintTwoFriction;


double ChLcpConstraintTwoFriction::Violation(double mc_i)
{
	return 0.0; //***TO DO*** compute true violation when in sticking?
}



void ChLcpConstraintTwoFriction::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoBodies::StreamOUT(mstream);

		// stream out all member data
	mstream << friction;
}
 
void ChLcpConstraintTwoFriction::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoBodies::StreamIN(mstream);

		// stream in all member data
	mstream >> friction;
}




} // END_OF_NAMESPACE____


