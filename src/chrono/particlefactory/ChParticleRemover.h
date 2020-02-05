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

    /// easy access to box of trigger
    geometry::ChBox& GetBox() {
        auto mtrigbox = std::dynamic_pointer_cast<ChParticleEventTriggerBox>(trigger);
        if (!mtrigbox)
            throw ChException("ChParticleRemoverBox had trigger replaced to non-box type");

        return mtrigbox->mbox;
    }

    /// easy access to in/out toggle of trigger
    void SetRemoveOutside(bool minvert) {
        if (auto mtrigbox = std::dynamic_pointer_cast<ChParticleEventTriggerBox>(trigger)) {
            mtrigbox->SetTriggerOutside(minvert);
        } else {
            throw ChException("ChParticleRemoverBox had trigger replaced to non-box type");
        }
    }
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
