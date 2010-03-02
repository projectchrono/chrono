///////////////////////////////////////////////////
//
//   ChLcpConstraintNodeFrictionT.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintNodeFrictionT.h"


namespace chrono
{

// Register into the object factory, to enable run-time 
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintNodeFrictionT> a_registration_ChLcpConstraintNodeFrictionT;


double ChLcpConstraintNodeFrictionT::Violation(double mc_i)
{
	return 0.0; //***TO DO*** compute true violation when in sticking?
}



void ChLcpConstraintNodeFrictionT::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoGeneric::StreamOUT(mstream);

}
 
void ChLcpConstraintNodeFrictionT::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoGeneric::StreamIN(mstream);

}




} // END_OF_NAMESPACE____


