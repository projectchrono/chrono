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

#include <list>

#include "chrono_parallel/collision/ChContactContainerParallel.h"

namespace chrono {

/// @addtogroup parallel_collision
/// @{

/// Specialization of the parallel contact container for SMC contacts.
class CH_PARALLEL_API ChContactContainerParallelSMC : public ChContactContainerParallel {
  public:
    ChContactContainerParallelSMC(ChParallelDataManager* dc);
    ChContactContainerParallelSMC(const ChContactContainerParallelSMC& other);

    /// "Virtual" copy constructor (covariant return type).
    virtual ChContactContainerParallelSMC* Clone() const override { return new ChContactContainerParallelSMC(*this); }

    virtual void BeginAddContact() override;
    virtual void EndAddContact() override;

    /// Add a contact between two collision shapes, storing it into this container.
    /// A compositecontact material is created from the two given materials.
    /// In this case, the collision info object may have null pointers to collision shapes.
    virtual void AddContact(const collision::ChCollisionInfo& cinfo,
                            std::shared_ptr<ChMaterialSurface> mat1,
                            std::shared_ptr<ChMaterialSurface> mat2) override;

    /// Add a contact between two collision shapes, storing it into this container.
    /// The collision info object is assumed to contain valid pointers to the two colliding shapes.
    /// A composite contact material is created from their material properties.
    virtual void AddContact(const collision::ChCollisionInfo& cinfo) override;

    /// Process the contact between the two specified collision shapes on the two specified bodies
    /// (compute composite material properties and load in global data structure).
    virtual void AddContact(int index, int b1, int s1, int b2, int s2) override;
};

/// @} parallel_colision

}  // end namespace chrono
