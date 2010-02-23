///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoContact.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver 
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoContact.h" 


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoContact> a_registration_ChLcpConstraintTwoContact;


 
void ChLcpConstraintTwoContact::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoBodies::StreamOUT(mstream);

		// stream out all member data..

}

void ChLcpConstraintTwoContact::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoBodies::StreamIN(mstream);

		// stream in all member data..

}




} // END_OF_NAMESPACE____


