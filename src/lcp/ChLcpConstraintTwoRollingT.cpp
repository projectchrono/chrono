///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoRollingT.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoRollingT.h"


namespace chrono
{

// Register into the object factory, to enable run-time 
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoRollingT> a_registration_ChLcpConstraintTwoRollingT;


double ChLcpConstraintTwoRollingT::Violation(double mc_i)
{
	return 0.0; //***TO DO*** compute true violation when in sticking?
}



void ChLcpConstraintTwoRollingT::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoBodies::StreamOUT(mstream);

}
 
void ChLcpConstraintTwoRollingT::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoBodies::StreamIN(mstream);

}




} // END_OF_NAMESPACE____


