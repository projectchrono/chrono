//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//
// File author: A.Tasora

#ifndef CHPARTICLEPROCESSEVENT_H
#define CHPARTICLEPROCESSEVENT_H

#include "chrono/physics/ChSystem.h"
#include "chrono/particlefactory/ChParticleEventTrigger.h"

namespace chrono {
namespace particlefactory {

/// BASE class for all event processors of single particles
/// You can directly use the ready-to-use processor for basic
/// behaviors (remove particle, count particle, etc.),
/// or inherit your own class with custom event processing.
class ChParticleProcessEvent {
  public:
    /// Children class MUST implement this function according to their
    /// desired behavior. For example, one class might be used to delete the
    /// particles when event is triggered, so here it will remove the particle, etc.
    virtual void ParticleProcessEvent(std::shared_ptr<ChBody> mbody,
                                      ChSystem& msystem,
                                      std::shared_ptr<ChParticleEventTrigger> mprocessor) = 0;

    /// Children classes might optionally implement this.
    /// The ChParticleProcessor will call this once, before each ProcessParticles()
    virtual void SetupPreProcess(ChSystem& msystem){};
    /// Children classes might optionally implement this.
    /// The ChParticleProcessor will call this once, after each ProcessParticles()
    virtual void SetupPostProcess(ChSystem& msystem){};
};

/// Simpliest case: no event processing
/// Just an example.
class ChParticleProcessEventDoNothing : public ChParticleProcessEvent {
  public:
    /// Do nothing on process
    virtual void ParticleProcessEvent(std::shared_ptr<ChBody> mbody,
                                      ChSystem& msystem,
                                      std::shared_ptr<ChParticleEventTrigger> mprocessor){};
};

/// Processed particle will be removed.
/// Note that this does not necessarily means also deletion of the particle,
/// because they are handled with shared pointers; however if they were
/// referenced only by the ChSystem, this also leads to deletion.
class ChParticleProcessEventRemove : public ChParticleProcessEvent {
  private:
    std::list<std::shared_ptr<ChBody> > to_delete;

  public:
    /// Remove the particle from the system.
    virtual void ParticleProcessEvent(std::shared_ptr<ChBody> mbody,
                                      ChSystem& msystem,
                                      std::shared_ptr<ChParticleEventTrigger> mprocessor) {
        // msystem.Remove(mbody);
        to_delete.push_back(mbody);
    }

    virtual void SetupPreProcess(ChSystem& msystem) { to_delete.clear(); }

    virtual void SetupPostProcess(ChSystem& msystem) {
        std::list<std::shared_ptr<ChBody> >::iterator ibody = to_delete.begin();
        while (ibody != to_delete.end()) {
            msystem.Remove((*ibody));
            ++ibody;
        }
    }
};

/// Processed particle will be counted.
/// Note that you have to use this processor with triggers that
/// make some sense, such as ChParticleEventFlowInRectangle, because
/// with others like ChParticleEventTriggerBox you'll get a trigger per
/// each timestep, not only after the particle entered the box.
class ChParticleProcessEventCount : public ChParticleProcessEvent {
  public:
    ChParticleProcessEventCount() { counter = 0; }

    /// Remove the particle from the system.
    virtual void ParticleProcessEvent(std::shared_ptr<ChBody> mbody,
                                      ChSystem& msystem,
                                      std::shared_ptr<ChParticleEventTrigger> mprocessor) {
        ++counter;
    }

    int counter;
};

/// Processed particle will increment a mass counter .
/// Note that you have to use this processor with triggers that
/// make some sense, such as ChParticleEventFlowInRectangle, because
/// with others like ChParticleEventTriggerBox you'll get a trigger per
/// each timestep, not only after the particle entered the box.
class ChParticleProcessEventMassCount : public ChParticleProcessEvent {
  public:
    ChParticleProcessEventMassCount() { counted_mass = 0; }

    /// Remove the particle from the system.
    virtual void ParticleProcessEvent(std::shared_ptr<ChBody> mbody,
                                      ChSystem& msystem,
                                      std::shared_ptr<ChParticleEventTrigger> mprocessor) {
        counted_mass += mbody->GetMass();
    }

    double counted_mass;
};

/// Processed particle will increment a NxM matrix mass counter,
/// so that a statistical distribution of flow over a uv surface
/// can be obtained. Note that the matrix maps the 0...1 of the u (matrix rows)
/// and the 0..1 of the v (matrix columns) of the uv surface.
/// Note: For the moment, this supports only the case of trigger of type
/// ChParticleEventFlowInRectangle.
class ChParticleProcessEventMassDistribution : public ChParticleProcessEvent {
  public:
    ChParticleProcessEventMassDistribution(int u_sects = 10, int v_sects = 10) { mmass.Reset(u_sects, v_sects); }

    /// Remove the particle from the system.
    virtual void ParticleProcessEvent(std::shared_ptr<ChBody> mbody,
                                      ChSystem& msystem,
                                      std::shared_ptr<ChParticleEventTrigger> mprocessor) {
        assert(std::dynamic_pointer_cast<ChParticleEventFlowInRectangle>(mprocessor));

        if (auto mrectangleprocessor = std::dynamic_pointer_cast<ChParticleEventFlowInRectangle>(mprocessor)) {
            int irow = (int)floor(mmass.GetRows() * mrectangleprocessor->last_intersectionUV.x());
            if (irow >= mmass.GetRows())
                irow = mmass.GetRows() - 1;
            int icol = (int)floor(mmass.GetColumns() * mrectangleprocessor->last_intersectionUV.y());
            if (icol >= mmass.GetColumns())
                icol = mmass.GetColumns() - 1;

            mmass(irow, icol) += mbody->GetMass();
        }
    }

    ChMatrixDynamic<> mmass;
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
