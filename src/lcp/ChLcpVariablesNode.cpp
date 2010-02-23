///////////////////////////////////////////////////
//
//   ChLcpVariablesNode.cpp
//
//
//    file for CHRONO HYPEROCTANT LCP solver
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
 

#include "ChLcpVariablesNode.h"
 

namespace chrono 
{



ChLcpVariablesNode& ChLcpVariablesNode::operator=(const ChLcpVariablesNode& other)
{
	if (&other == this) return *this;

	// copy parent class data
	ChLcpVariables::operator=(other);

	// copy class data
	user_data = other.user_data;
	mass = other.mass;

	return *this;
}



// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegister<ChLcpVariablesNode> a_registration_ChLcpVariablesNode;



} // END_OF_NAMESPACE____


