#ifndef CHNODEFEMBASE_H
#define CHNODEFEMBASE_H

//////////////////////////////////////////////////
//
//   ChNodeFEMbase.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include "physics/ChIndexedNodes.h"


namespace chrono
{
namespace fem
{


/// Base class for a generic finite element node
/// that can be stored in ChMesh containers.
/// Children classes must implement specialized versions.

class ChApi ChNodeFEMbase  :  public chrono::ChNodeXYZ
{
public:

				/// Set the rest position as the actual position.
	virtual void Relax () =0;

};





} // END_OF_NAMESPACE____
} // END_OF_NAMESPACE____


#endif






