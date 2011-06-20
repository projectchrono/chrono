#ifndef CHBODYGPU_H
#define CHBODYGPU_H

//////////////////////////////////////////////////
//
//   ChBodyGPU.h
//
//   Derived Class for GPU rigid bodies, 
//   
//   
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////
#include "physics/ChBody.h"
#include "ChApiGPU.h"
#include "ChCCollisionModelGPU.h"
namespace chrono
{

	using namespace collision;


	class ChApiGPU ChBodyGPU : public ChBody {

		CH_RTTI(ChBodyGPU,ChPhysicsItem);

	public:

		//
		// CONSTRUCTORS
		//
		/// Build a rigid body.
		ChBodyGPU ();
		/// Destructor
		~ChBodyGPU ();
		virtual ChCollisionModel* InstanceCollisionModel();
		int id;
	};

	typedef ChSharedPtr<ChBodyGPU> ChSharedBodyGPUPtr;

} // END_OF_NAMESPACE____


#endif
