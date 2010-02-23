///////////////////////////////////////////////////
//
//   ChLcpConstraintTwoFrictionApprox.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "ChLcpConstraintTwoFrictionApprox.h" 


namespace chrono
{

// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpConstraintTwoFrictionApprox> a_registration_ChLcpConstraintTwoFrictionApprox;



void ChLcpConstraintTwoFrictionApprox::Project()
{
	if (!constraint_Tother)
		return;

	if (!constraint_N)
		return;

	// METHOD
	// Tasora horizontal projection on cone (not necessarily a contractive mapping)
	double f_tang = sqrt (l_i*l_i + constraint_Tother->l_i * constraint_Tother->l_i );
	double f_limit = friction * constraint_N->Get_l_i();
	if (f_tang > f_limit)
	{	
		double shrink = f_limit / f_tang;
		l_i *= shrink;
		//constraint_Tother->l_i *= shrink;

	}

}



void ChLcpConstraintTwoFrictionApprox::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChLcpConstraintTwoFriction::StreamOUT(mstream);

		// stream out all member data
	// NOTHING INTERESTING TO SERIALIZE (the Cq jacobians are not so
	// important to waste disk space.. they may be recomputed run-time,
	// and pointers to variables must be rebound in run-time.)
}
 
void ChLcpConstraintTwoFrictionApprox::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChLcpConstraintTwoFriction::StreamIN(mstream);

		// stream in all member data
	// NOTHING INTERESTING TO DESERIALIZE (the Cq jacobians are not so
	// important to waste disk space.. they may be recomputed run-time,
	// and pointers to variables must be rebound in run-time.)
}




} // END_OF_NAMESPACE____


