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

#ifndef CHPARTICLEREMOVER_H
#define CHPARTICLEREMOVER_H

#include "ChParticleProcessor.h"

namespace chrono {
namespace particlefactory {

/// Utility class: shortcut for creating a ChParticleProcessor
/// that already contains a ChParticleEventTriggerBox and
/// a ChParticleProcessEventRemove. Keeps things easier.
class ChParticleRemoverBox : public ChParticleProcessor {
  public:
    ChParticleRemoverBox() {
        this->SetEventTrigger(ChSharedPtr<ChParticleEventTriggerBox>(new ChParticleEventTriggerBox));
        this->SetParticleEventProcessor(ChSharedPtr<ChParticleProcessEventRemove>(new ChParticleProcessEventRemove));
    }

    /// easy access to box of trigger
    geometry::ChBox& GetBox() {
        ChSharedPtr<ChParticleEventTriggerBox> mtrigbox = trigger.DynamicCastTo<ChParticleEventTriggerBox>();
        if (mtrigbox.IsNull())
            throw ChException("ChParticleRemoverBox had trigger replaced to non-box type");
        return mtrigbox->mbox;
    }

    /// easy access to in/out toggle of trigger
    void SetRemoveOutside(bool minvert) {
        ChSharedPtr<ChParticleEventTriggerBox> mtrigbox = trigger.DynamicCastTo<ChParticleEventTriggerBox>();
        if (mtrigbox.IsNull())
            throw ChException("ChParticleRemoverBox had trigger replaced to non-box type");
        mtrigbox->SetTriggerOutside(minvert);
    }
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif
