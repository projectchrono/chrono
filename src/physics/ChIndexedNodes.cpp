///////////////////////////////////////////////////
//
//   ChIndexedNodes.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "physics/ChIndexedNodes.h"
#include "physics/ChSystem.h"

#include "physics/ChExternalObject.h"
#include "collision/ChCModelBulletNode.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChIndexedNodes> a_registration_ChIndexedNodes;

	

//////////////////////////////////////
//////////////////////////////////////

/// BASE CLASS FOR NODES


ChNodeBase::ChNodeBase()
{
}

ChNodeBase::~ChNodeBase()
{
}

ChNodeBase::ChNodeBase (const ChNodeBase& other) 
{
}

ChNodeBase& ChNodeBase::operator= (const ChNodeBase& other)
{
	if (&other == this) 
		return *this;

	return *this;
}


//////////////////////////////////////

/// CLASS FOR A PARTICLE


ChNodeXYZ::ChNodeXYZ()
{
	this->pos = VNULL;
	this->pos_dt = VNULL;
	this->pos_dtdt = VNULL;
}

ChNodeXYZ::~ChNodeXYZ()
{
}

ChNodeXYZ::ChNodeXYZ (const ChNodeXYZ& other) :
					ChNodeBase(other) 
{
	this->pos = other.pos;
	this->pos_dt = other.pos_dt;
	this->pos_dtdt = other.pos_dtdt;
}

ChNodeXYZ& ChNodeXYZ::operator= (const ChNodeXYZ& other)
{
	if (&other == this) 
		return *this;

	ChNodeBase::operator=(other);

	this->pos = other.pos;
	this->pos_dt = other.pos_dt;
	this->pos_dtdt = other.pos_dtdt;

	return *this;
}



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR INDEXED NODES CONTAINER


ChIndexedNodes::ChIndexedNodes ()
{
	SetIdentifier(ChGLOBALS().GetUniqueIntID()); // mark with unique ID
}


ChIndexedNodes::~ChIndexedNodes ()
{

}



//////// FILE I/O

void ChIndexedNodes::StreamOUT(ChStreamOutBinary& mstream)
{
		// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data

}

void ChIndexedNodes::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// stream in all member data
}






} // END_OF_NAMESPACE____


/////////////////////
