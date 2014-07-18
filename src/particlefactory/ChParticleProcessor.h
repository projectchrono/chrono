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

#ifndef CHPARTICLEPROCESSOR_H
#define CHPARTICLEPROCESSOR_H

#include "core/ChSmartpointers.h"
#include "ChParticleEventTrigger.h"
#include "ChParticleProcessEvent.h"


namespace particlefactory
{



	/// Class that can be used to process particles.
	/// It uses two 'tools' that can be plugged in: 
	/// 1) an object inherited from ChParticleEventTrigger,
	/// that decides IF an event must be triggered (there are ready-to-use triggers
	/// such as trigger if inside a box, if older than X, etc., or you can implement your own)
	/// 2) an object inherited from ChParticleProcessEvent,
	/// the performs an action if the event is triggered (there are ready-to-use event
	/// processors such as remove particle, etc.)
	/// Note: by default, the default trigger is ChParticleEventTriggerNever and
	/// the default particle event processor is ChParticleProcessEventDoNothing, so
	/// the default behavior is 'do nothing', so you must plug in more sophisticated ones
	/// after you create the ChParticleProcessor and before you use it.
class ChParticleProcessor : public ChShared
{
public:

	ChParticleProcessor() 
	{
		// default trigger: trigger never
		trigger = ChSharedPtr<ChParticleEventTriggerNever> (new ChParticleEventTriggerNever);
		// default event processor: do nothing
		particle_processor = ChSharedPtr<ChParticleProcessEventDoNothing> (new ChParticleProcessEventDoNothing);
	}

			/// This function process particles according to some rule,
			/// defined by plugging appropriate ChParticleEventTrigger
			/// and ChParticleProcessEvent.
			/// Returns the number of processed particles (those that
			/// triggered events.)
	virtual int ProcessParticles(ChSystem& msystem) 
	{
		this->trigger->SetupPreProcess(msystem);
		this->particle_processor->SetupPreProcess(msystem);

		int nprocessed = 0;

		ChSystem::IteratorBodies myiter = msystem.IterBeginBodies();
		while (myiter != msystem.IterEndBodies())
		{
			if (this->trigger->TriggerEvent((*myiter), msystem))
			{
					this->particle_processor->ParticleProcessEvent((*myiter), msystem, this->trigger);
					++nprocessed;
			}

			++myiter;
		}

		this->particle_processor->SetupPostProcess(msystem);
		this->trigger->SetupPostProcess(msystem);

		return nprocessed;
	}

		/// Use this function to plug in an event trigger.
	void SetEventTrigger ( ChSharedPtr<ChParticleEventTrigger> mtrigger) { trigger = mtrigger;}

		/// Use this function to plug in a particle event processor.
	void SetParticleEventProcessor ( ChSharedPtr<ChParticleProcessEvent> mproc) { particle_processor = mproc;}

protected:
	ChSharedPtr<ChParticleEventTrigger> trigger;
	ChSharedPtr<ChParticleProcessEvent> particle_processor;
};



} // end of namespace


#endif  
