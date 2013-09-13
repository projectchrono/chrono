///////////////////////////////////////////////////
//
//   ChElementSpring.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////



#include "ChElementSpring.h"


namespace chrono
{
namespace fem
{



ChElementSpring::ChElementSpring()
{
	spring_k = 1.0; 
	damper_r = 0.01; 

	nodes.resize(2);
}


ChElementSpring::~ChElementSpring()
{
}



} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____








