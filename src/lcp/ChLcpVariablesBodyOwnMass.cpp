///////////////////////////////////////////////////
//
//   ChLcpVariablesBodyOwnMass.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesBodyOwnMass.h"
 

namespace chrono 
{



ChLcpVariablesBodyOwnMass& ChLcpVariablesBodyOwnMass::operator=(const ChLcpVariablesBodyOwnMass& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariablesBody::operator=(other);

	// copy class data
	mass= other.mass;
	inv_mass = other.inv_mass;

	inertia = other.inertia;
	inv_inertia = other.inv_inertia;

	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesBodyOwnMass> a_registration_ChLcpVariablesBodyOwnMass;



} // END_OF_NAMESPACE____


