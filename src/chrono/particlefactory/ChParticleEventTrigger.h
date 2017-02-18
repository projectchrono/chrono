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

#ifndef CHPARTICLEEVENTTRIGGER_H
#define CHPARTICLEEVENTTRIGGER_H

#include <unordered_map>

#include "chrono/physics/ChSystem.h"
#include "chrono/geometry/ChBox.h"

namespace chrono {
namespace particlefactory {

/// BASE class for event triggers for the ChParticleProcessor
/// You can directly use the ready-to-use triggers for common
/// triggering (particle collides with some object, particle inside
/// a box, etc.), or inherit your own class with custom triggering.
class ChParticleEventTrigger {
  public:
    /// Children classes MUST implement this.
    /// Return true means that a ChParticleProcessEvent must
    /// be done, return false means that no ChParticleProcessEvent must be done.
    virtual bool TriggerEvent(std::shared_ptr<ChBody> mbody, ChSystem& msystem) = 0;

    /// Children classes might optionally implement this.
    /// The ChParticleProcessor will call this once, before each ProcessParticles()
    virtual void SetupPreProcess(ChSystem& msystem){};
    /// Children classes might optionally implement this.
    /// The ChParticleProcessor will call this once, after each ProcessParticles()
    virtual void SetupPostProcess(ChSystem& msystem){};
};

/// Simpliest case: never trigger
class ChParticleEventTriggerNever : public ChParticleEventTrigger {
  public:
    /// Never trig events
    virtual bool TriggerEvent(std::shared_ptr<ChBody> mbody, ChSystem& msystem) { return false; }
};

/// This can be used to trigger events when particles are
/// inside a box volume. Event is triggered all the times
/// that the particle processor is run, not only the 1st time
/// that particle enters the volume.
/// Only the center of gravity of particle is taken
/// into consideration, regardless of its size.
class ChParticleEventTriggerBox : public ChParticleEventTrigger {
  public:
    ChParticleEventTriggerBox() { invert_volume = false; }

    /// This function triggers the a particle event according to the fact
    /// the the particle is inside a box.
    /// If SetTriggerOutside(true), viceversa triggers event outside the box.
    virtual bool TriggerEvent(std::shared_ptr<ChBody> mbody, ChSystem& msystem) {
        ChVector<> particle_pos = mbody->GetPos();

        ChVector<> localpos = mbox.Pos + mbox.Rot * particle_pos;

        if (((fabs(localpos.x()) < mbox.Size.x()) && (fabs(localpos.y()) < mbox.Size.y()) && (fabs(localpos.z()) < mbox.Size.z())) ^
            invert_volume)  // XOR
            return true;
        else
            return false;
    }

    void SetTriggerOutside(bool minvert) { invert_volume = minvert; }

    geometry::ChBox mbox;

  protected:
    bool invert_volume;
};

class _particle_last_pos {
  public:
    _particle_last_pos();
    _particle_last_pos(const _particle_last_pos& source) { mbody = source.mbody, mpos = source.mpos; }
    _particle_last_pos(std::shared_ptr<ChBody> mb, ChVector<> mp) : mbody(mb), mpos(mp){};

    std::shared_ptr<ChBody> mbody;
    ChVector<> mpos;
};

/// Trigger an event each time a particle flows into a rectangle.
/// Rectangle is defined with a coordinate system, in the center, and two X Y sizes.
/// This is triggered only if the particle is flowing from positove Z to negative Z.
/// Note! there is a 'margin' parameter that must be greater than zero: only particles that
/// in the neighborhood of the rectangle (less distant than the margin) are cosidered, to
/// save computational time. Too small margin might miss some fast-travelin particles, so
/// margin shoud be at least 'max particle speed' * dt.
class ChParticleEventFlowInRectangle : public ChParticleEventTrigger {
  public:
    ChParticleEventFlowInRectangle(double mXsize = 1, double mYsize = 1) {
        Xsize = mXsize;
        Ysize = mYsize;
        margin = 0.1 * ChMin(Xsize, Ysize);
    }

    /// This function triggers the a particle event according to the fact
    /// the the particle is crossing a rectangle.
    virtual bool TriggerEvent(std::shared_ptr<ChBody> mbody, ChSystem& msystem) {
        ChVector<> particle_pos = mbody->GetPos();
        ChVector<> localpos = rectangle_csys.TransformParentToLocal(mbody->GetPos());

        // Is in lower part of rectangle?
        if ((localpos.z() <= 0) && (-localpos.z() < margin) && (fabs(localpos.x()) < 0.5 * Xsize + margin) &&
            (fabs(localpos.y()) < 0.5 * Ysize + margin)) {
            // Was it in the upper part, at last iteration?
            std::unordered_map<size_t, _particle_last_pos>::iterator mcached = last_positions.find((size_t)mbody.get());
            if (mcached != last_positions.end()) {
                ChVector<> pA = (*mcached).second.mpos;
                ChVector<> pB = localpos;
                ChVector<> pColl;
                double tb = pA.z() / (pA.z() - pB.z());
                double ta = -pB.z() / (pA.z() - pB.z());
                if ((pA.z() == 0) && (pB.z() == 0))
                    pColl = pA;
                else
                    pColl = pA * ta + pB * tb;

                if ((fabs(pColl.x()) < 0.5 * Xsize) && (fabs(pColl.y()) < 0.5 * Ysize)) {
                    last_intersectionUV.x() = (pColl.x() + 0.5 * Xsize) / Xsize;
                    last_intersectionUV.y() = (pColl.y() + 0.5 * Ysize) / Ysize;
                    last_intersectionUV.z() = 0;
                    return true;
                }
            }
        }

        return false;
    }

    virtual void SetupPostProcess(ChSystem& msystem) {
        last_positions.clear();

        ChSystem::IteratorBodies myiter = msystem.IterBeginBodies();
        while (myiter != msystem.IterEndBodies()) {
            ChVector<> localpos = rectangle_csys.TransformParentToLocal((*myiter)->GetPos());
            if ((localpos.z() > 0) && (localpos.z() < margin) && (fabs(localpos.x()) < 0.5 * Xsize + margin) &&
                (fabs(localpos.y()) < 0.5 * Ysize + margin)) {
                // ok, was in the upper part Z>0 of the triangle.. store in hash table for next
                // run, so that one will know if the particle crossed the rectangle into Z<0
                _particle_last_pos mlastpos((*myiter), localpos);
                last_positions.insert({(size_t)(*myiter).get(), mlastpos});
            }

            ++myiter;
        }
    };

    double Xsize;
    double Ysize;
    double margin;
    ChCoordsys<> rectangle_csys;

    ChVector<> last_intersectionUV;  // .x and .y in range 0..1

  protected:
    std::unordered_map<size_t, _particle_last_pos> last_positions;
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
