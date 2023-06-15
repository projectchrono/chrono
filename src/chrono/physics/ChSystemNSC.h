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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Physical system in which contact is modeled using a non-smooth
// (complementarity-based) method.
//
// =============================================================================

#ifndef CH_SYSTEM_NSC_H
#define CH_SYSTEM_NSC_H

#include "chrono/physics/ChSystem.h"

namespace chrono {

/// Class for a physical system in which contact is modeled using a non-smooth
/// (complementarity-based) method.
class ChApi ChSystemNSC : public ChSystem {

  public:
    /// Create a physical system.
    /// If init_sys is false, the collision system oand solver are not initialized.
    ChSystemNSC(bool init_sys = true);

    /// Copy constructor
    ChSystemNSC(const ChSystemNSC& other);

    /// Destructor
    virtual ~ChSystemNSC() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChSystemNSC* Clone() const override { return new ChSystemNSC(*this); }

    /// Return the contact method supported by this system.
    virtual ChContactMethod GetContactMethod() const override final { return ChContactMethod::NSC; }

    /// Replace the contact container.
    virtual void SetContactContainer(std::shared_ptr<ChContactContainer> container) override;

    // SERIALIZATION

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChSystemNSC, 0)

}  // end namespace chrono

#endif
