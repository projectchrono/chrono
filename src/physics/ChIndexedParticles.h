#ifndef CHINDEXEDPARTICLES_H
#define CHINDEXEDPARTICLES_H

//////////////////////////////////////////////////
//
//   ChIndexedParticles.h
//
//   Interface class for clusters of 'particles' that can
//   be accessed with an index. Particles have 6 DOF.
//   Must be inherited by children classes.
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
// 	 Copyright:Alessandro Tasora / DeltaKnowledge
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////


#include <math.h>

#include "core/ChFrameMoving.h"
#include "physics/ChPhysicsItem.h"

namespace chrono
{

using namespace collision;



// Forward references (for parent hierarchy pointer)

class ChSystem;


/// Base class for a single particle to be used in ChIndexedParticles containers.
/// It is an item that has 6 degrees of freedom, like a moving frame.

class ChParticleBase : public ChFrameMoving<double>  
{
public:
	ChParticleBase();
	virtual ~ChParticleBase();

	ChParticleBase (const ChParticleBase& other); // Copy constructor
	ChParticleBase& operator= (const ChParticleBase& other); //Assignment operator

			// Access the 'LCP variables' of the node
	virtual ChLcpVariables& Variables() =0; 
};


/// Interface class for clusters of particles that can
/// be accessed with an index.
/// Must be inherited by children classes.

class ChIndexedParticles : public ChPhysicsItem 
{
						// Chrono simulation of RTTI, needed for serialization
	CH_RTTI(ChIndexedParticles,ChPhysicsItem);

private:
			//
	  		// DATA
			//

public:

			//
	  		// CONSTRUCTORS
			//

				/// Build a cluster of particles.
				/// By default the cluster will contain 0 particles.
	ChIndexedParticles ();

				/// Destructor
	~ChIndexedParticles ();



			//
	  		// FUNCTIONS
			//

				/// Get the number of particles
	virtual unsigned int GetNparticles() =0;

				/// Access the N-th particle 
	virtual ChParticleBase& GetParticle(unsigned int n) =0;
				
				/// Resize the particle cluster. Also clear the state of 
				/// previously created particles, if any.
	virtual void ResizeNparticles(int newsize) =0;

				/// Add a new particle to the particle cluster, passing a 
				/// coordinate system as initial state.
	virtual void AddParticle(ChCoordsys<double> initial_state = CSYSNORM) =0;

				/// Number of coordinates of the particle cluster
	virtual int GetDOF  ()   {return 6*GetNparticles();}



			//
			// STREAMING
			//


				/// Method to allow deserializing a persistent binary archive (ex: a file)
				/// into transient data.
	void StreamIN(ChStreamInBinary& mstream);

				/// Method to allow serializing transient data into a persistent
				/// binary archive (ex: a file).
	void StreamOUT(ChStreamOutBinary& mstream);


};




typedef ChSharedPtr<ChIndexedParticles> ChSharedIndexedParticlesPtr;



} // END_OF_NAMESPACE____


#endif
