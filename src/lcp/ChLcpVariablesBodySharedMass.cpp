///////////////////////////////////////////////////
//
//   ChLcpVariablesBodySharedMass.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesBodySharedMass.h"
 

namespace chrono 
{



ChLcpVariablesBodySharedMass& ChLcpVariablesBodySharedMass::operator=(const ChLcpVariablesBodySharedMass& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariablesBody::operator=(other);

	// copy class data
	sharedmass = other.sharedmass;

	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesBodySharedMass> a_registration_ChLcpVariablesBodySharedMass;



} // END_OF_NAMESPACE____


