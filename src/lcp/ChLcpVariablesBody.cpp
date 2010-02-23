///////////////////////////////////////////////////
//
//   ChLcpVariablesBody.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesBody.h"
 

namespace chrono 
{



ChLcpVariablesBody& ChLcpVariablesBody::operator=(const ChLcpVariablesBody& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariables::operator=(other);

	// copy class data
	user_data = other.user_data;

	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChLcpVariablesBody> a_registration_ChLcpVariablesBody;



} // END_OF_NAMESPACE____


