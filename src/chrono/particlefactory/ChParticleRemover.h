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

#ifndef CHPARTICLEREMOVER_H
#define CHPARTICLEREMOVER_H

#include "chrono/particlefactory/ChParticleProcessor.h"

namespace chrono {
namespace particlefactory {

/// Utility class: shortcut for creating a ChParticleProcessor
/// that already contains a ChParticleEventTriggerBox and
/// a ChParticleProcessEventRemove. Keeps things easier.
class ChParticleRemoverBox : public ChParticleProcessor {
  public:
    ChParticleRemoverBox() {
        this->SetEventTrigger(std::shared_ptr<ChParticleEventTriggerBox>(new ChParticleEventTriggerBox));
        this->SetParticleEventProcessor(std::shared_ptr<ChParticleProcessEventRemove>(new ChParticleProcessEventRemove));
    }

    /// Set the dimensions and position of the trigger box.
    void SetBox(const ChVector<>& lengths, const ChFrame<>& frame) {
        auto trigbox = std::dynamic_pointer_cast<ChParticleEventTriggerBox>(trigger);
        if (!trigbox)
            throw ChException("ChParticleRemoverBox had trigger replaced to non-box type");
        trigbox->m_box.SetLengths(lengths);
        trigbox->m_frame = frame;
    }

    geometry::ChBox& GetBox() {
        auto trigbox = std::dynamic_pointer_cast<ChParticleEventTriggerBox>(trigger);
        if (!trigbox)
            throw ChException("ChParticleRemoverBox had trigger replaced to non-box type");
        return trigbox->m_box;
    }

    /// Toggle inside/outside trigger.
    void SetRemoveOutside(bool invert) {
        auto trigbox = std::dynamic_pointer_cast<ChParticleEventTriggerBox>(trigger);
        if (!trigbox)
            throw ChException("ChParticleRemoverBox had trigger replaced to non-box type");
        trigbox->SetTriggerOutside(invert);
    }
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
