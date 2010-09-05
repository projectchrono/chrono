///////////////////////////////////////////////////
//
//   ChParticles.cpp
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

    
#include <stdlib.h>
#include <algorithm>

#include "core/ChTrasform.h"
#include "physics/ChIndexedParticles.h"
#include "physics/ChSystem.h"
#include "physics/ChGlobal.h"

#include "physics/ChExternalObject.h"
#include "collision/ChCModelBulletParticle.h"
#include "core/ChLinearAlgebra.h"

#include "core/ChMemory.h" // must be last include (memory leak debugger). In .cpp only.

namespace chrono
{

using namespace collision;
using namespace geometry;


// Register into the object factory, to enable run-time
// dynamic creation and persistence
ChClassRegisterABSTRACT<ChIndexedParticles> a_registration_ChIndexedParticles;

	

//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR A PARTICLE


ChParticleBase::ChParticleBase()
{
}

ChParticleBase::~ChParticleBase()
{
}

ChParticleBase::ChParticleBase (const ChParticleBase& other) :
					ChFrameMoving<double>(other)
{
}

ChParticleBase& ChParticleBase::operator= (const ChParticleBase& other)
{
	if (&other == this) 
		return *this;

	// parent class copy
	ChFrameMoving<double>::operator=(other);
	
	return *this;
}



//////////////////////////////////////
//////////////////////////////////////

/// CLASS FOR PARTICLE CLUSTER


ChIndexedParticles::ChIndexedParticles ()
{
	
}


ChIndexedParticles::~ChIndexedParticles ()
{
	
}



//////// FILE I/O

void ChIndexedParticles::StreamOUT(ChStreamOutBinary& mstream)
{
			// class version number
	mstream.VersionWrite(1);

		// serialize parent class too
	ChPhysicsItem::StreamOUT(mstream);

		// stream out all member data

}

void ChIndexedParticles::StreamIN(ChStreamInBinary& mstream)
{
		// class version number
	int version = mstream.VersionRead();

		// deserialize parent class too
	ChPhysicsItem::StreamIN(mstream);

		// stream in all member data

}






} // END_OF_NAMESPACE____


/////////////////////
