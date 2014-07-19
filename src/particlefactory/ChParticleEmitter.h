//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be 
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHPARTICLEEMITTER_H
#define CHPARTICLEEMITTER_H


#include "ChRandomShapeCreator.h"
#include "ChRandomParticlePosition.h"
#include "ChRandomParticleAlignment.h"
#include "ChRandomParticleVelocity.h"
#include "core/ChMathematics.h"
#include "core/ChVector.h"
#include "core/ChMatrix.h"
#include "core/ChDistribution.h"
#include "core/ChSmartpointers.h"
#include "physics/ChSystem.h"


namespace chrono {

/// Namespace for helper classes that build a system for generating
/// flows of particles. This system heavily relies on statistical
/// approaches, for example positions, rotations, shapes of particles
/// can be generated according to statistical distributions.
namespace particlefactory {


	
	/// Class for emitters of particles, with random positions,
	/// rotations, and random shapes. You can setup a variety of 
	/// different emitters by assembling different types of 
	/// items inherited by classes like ChRandomShapeCreator,
	/// ChRandomParticlePosition, etc.
class ChParticleEmitter
{
public:
	ChParticleEmitter() 
		{
			// defaults:
			particles_per_second = 100;
			particle_creator     = ChSharedPtr<ChRandomShapeCreatorSpheres>(new ChRandomShapeCreatorSpheres);
			particle_positioner  = ChSharedPtr<ChRandomParticlePositionRectangleOutlet>(new ChRandomParticlePositionRectangleOutlet);
			particle_aligner     = ChSharedPtr<ChRandomParticleAlignmentUniform> (new ChRandomParticleAlignmentUniform);
			particle_velocity    = ChSharedPtr<ChRandomParticleVelocity> (new ChRandomParticleVelocity);
			particle_angular_velocity = ChSharedPtr<ChRandomParticleVelocity> (new ChRandomParticleVelocity);
			creation_callback	 = 0;
			use_praticle_reservoir = false;
			particle_reservoir = 1000;
			created_particles	= 0;
		}

			/// Function that creates random particles with random shape, position
			/// and alignment each time it is called. 
			/// Typically, one calls this function once per timestep.
	void EmitParticles(ChSystem& msystem, double dt)
		{
			// get n.of particles to generate in this dt timestep, with floor roundoff
			int particles_per_step = (int)floor(dt*particles_per_second);
			// since int->double roundoff, adjust to have correct flow rate on large n. of steps
			if ((dt*particles_per_second - floor(dt*particles_per_second)) > ChRandom())
				particles_per_step++;

			// create the particles for this timestep
			for (int i = 0; i < particles_per_step; ++i)
			{
				if ((use_praticle_reservoir)&&(this->particle_reservoir <= 0))
					return;

				ChCoordsys<> mcoords;
				mcoords.pos = particle_positioner->RandomPosition();
				mcoords.rot = particle_aligner->RandomAlignment();

				ChSharedPtr<ChBody> mbody = particle_creator->RandomGenerateAndCallbacks(mcoords);
				
				mbody->SetPos_dt(particle_velocity->RandomVelocity());
				mbody->SetWvel_par(particle_angular_velocity->RandomVelocity());

				if (this->creation_callback)
					this->creation_callback->PostCreation(mbody, mcoords, *particle_creator.get_ptr());

				msystem.Add(mbody);

				--this->particle_reservoir;
				++this->created_particles;
			}
		}

			/// Pass an object from a ChPostCreationCallback-inherited class if you want to 
			/// set additional stuff on each created particle (ex.set some random asset, set some random material, or such)
	void SetCallbackPostCreation(ChCallbackPostCreation* mcallback) {this->creation_callback = mcallback;}

			/// Set the particle creator, that is an object whose class is
			/// inherited from ChRandomShapeCreator
	void SetParticleCreator   (ChSharedPtr<ChRandomShapeCreator> mc) {particle_creator = mc;}
		
			/// Set the particle positioner, that generates different positions for each particle
	void SetParticlePositioner(ChSharedPtr<ChRandomParticlePosition> mc) {particle_positioner = mc;}

			/// Set the particle aligner, that generates different rotations for each particle
	void SetParticleAligner   (ChSharedPtr<ChRandomParticleAlignment> mc) {particle_aligner = mc;}

			/// Set the generator of particle velocities, that generates different initial speed for each particle
	void SetParticleVelocity   (ChSharedPtr<ChRandomParticleVelocity> mc) {particle_velocity = mc;}
			
			/// Set the generator of angular velocities, for different initial angular velocity for each particle
	void SetParticleAngularVelocity  (ChSharedPtr<ChRandomParticleVelocity> mc) {particle_angular_velocity = mc;}

			/// Access the flow rate, measured as n.of particles per second.
	double& ParticlesPerSecond() {return particles_per_second;}

			/// Turn on this to limit the limit on max amount of particles.
	void SetUseParticleReservoir(bool ml) {this->use_praticle_reservoir = ml;}

			/// Access the max number of particles to create - after this goes to 0, the creation stops.
			/// Remember to turn on this limit with SetLimitParticleAmount()
	int& ParticleReservoirAmount() {return particle_reservoir;}

private:
	double particles_per_second;
	ChSharedPtr<ChRandomShapeCreator>	   particle_creator;
	ChSharedPtr<ChRandomParticlePosition>  particle_positioner;
	ChSharedPtr<ChRandomParticleAlignment> particle_aligner;
	ChSharedPtr<ChRandomParticleVelocity>  particle_velocity;
	ChSharedPtr<ChRandomParticleVelocity>  particle_angular_velocity;
	ChCallbackPostCreation* creation_callback;
	
	int  particle_reservoir;
	bool use_praticle_reservoir;
	int  created_particles;
};


} // end of namespace particlefactory
} // end of namespace chrono


#endif  
