// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2016 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#pragma once

#include "chrono_multicore/collision/ChContactContainerMulticore.h"

namespace chrono {

/// @addtogroup multicore_collision
/// @{

/// Specialization of the multicore contact container for NSC contacts.
class CH_MULTICORE_API ChContactContainerMulticoreNSC : public ChContactContainerMulticore {
  public:
    ChContactContainerMulticoreNSC(ChMulticoreDataManager* dc);
    ChContactContainerMulticoreNSC(const ChContactContainerMulticoreNSC& other);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerMulticoreNSC* Clone() const override { return new ChContactContainerMulticoreNSC(*this); }

    virtual void BeginAddContact() override;
    virtual void EndAddContact() override;

    /// Add a contact between two collision shapes, storing it into this container.
    /// A compositecontact material is created from the two given materials.
    /// In this case, the collision info object may have null pointers to collision shapes.
    virtual void AddContact(const ChCollisionInfo& cinfo,
                            std::shared_ptr<ChMaterialSurface> mat1,
                            std::shared_ptr<ChMaterialSurface> mat2) override;

    /// Add a contact between two collision shapes, storing it into this container.
    /// The collision info object is assumed to contain valid pointers to the two colliding shapes.
    /// A composite contact material is created from their material properties.
    virtual void AddContact(const ChCollisionInfo& cinfo) override;

    /// Process the contact between the two specified collision shapes on the two specified bodies
    /// (compute composite material properties and load in global data structure).
    virtual void AddContact(int index, int b1, int s1, int b2, int s2) override;
};

/// @} multicore_colision

}  // end namespace chrono
