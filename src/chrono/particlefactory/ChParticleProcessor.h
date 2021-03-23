// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHPARTICLEPROCESSOR_H
#define CHPARTICLEPROCESSOR_H

#include "chrono/particlefactory/ChParticleEventTrigger.h"
#include "chrono/particlefactory/ChParticleProcessEvent.h"

namespace chrono {
namespace particlefactory {

/// @addtogroup chrono_particles
/// @{

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
class ChParticleProcessor {
  public:
    ChParticleProcessor() {
        // default trigger: trigger never
        trigger = chrono_types::make_shared<ChParticleEventTriggerNever>();
        // default event processor: do nothing
        particle_processor = chrono_types::make_shared<ChParticleProcessEventDoNothing>();
    }

    /// This function process particles according to some rule, defined by plugging appropriate ChParticleEventTrigger
    /// and ChParticleProcessEvent.
    /// Returns the number of processed particles (those that triggered events.)
    virtual int ProcessParticles(ChSystem& msystem) {
        this->trigger->SetupPreProcess(msystem);
        this->particle_processor->SetupPreProcess(msystem);

        int nprocessed = 0;

        for (auto body : msystem.Get_bodylist()) {
            if (this->trigger->TriggerEvent(body, msystem)) {
                this->particle_processor->ParticleProcessEvent(body, msystem, this->trigger);
                ++nprocessed;
            }
        }

        this->particle_processor->SetupPostProcess(msystem);
        this->trigger->SetupPostProcess(msystem);

        return nprocessed;
    }

    /// Use this function to plug in an event trigger.
    void SetEventTrigger(std::shared_ptr<ChParticleEventTrigger> mtrigger) { trigger = mtrigger; }

    /// Use this function to plug in a particle event processor.
    void SetParticleEventProcessor(std::shared_ptr<ChParticleProcessEvent> mproc) { particle_processor = mproc; }

  protected:
    std::shared_ptr<ChParticleEventTrigger> trigger;
    std::shared_ptr<ChParticleProcessEvent> particle_processor;
};

/// @} chrono_particles

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
