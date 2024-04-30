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

#ifndef CHLINKSCREW_H
#define CHLINKSCREW_H

#include "chrono/physics/ChLinkLock.h"

namespace chrono {

/// Screw joint between two rigid bodies. This
/// link type is able to couple translation and rotation.

class ChApi ChLinkLockScrew : public ChLinkLock {
  protected:
    double tau;  ///< transmission coeff.

  public:
    ChLinkLockScrew();
    ChLinkLockScrew(const ChLinkLockScrew& other);
    virtual ~ChLinkLockScrew() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkLockScrew* Clone() const override { return new ChLinkLockScrew(*this); }

    // Inherit the link-lock computations like it were a
    // normal "revolute" joint, but then modifies the Z-lock parts of C,
    // Cdt, Cdtdt, [Cq] etc., in order to have z = tau * alpha.
    virtual void UpdateState() override;

    double GetTransmissionRatio() const { return tau; };
    void SetTransmissionRatio(double mset) { tau = mset; }
    double GetThread() const { return tau * (2 * CH_PI); };
    void SetThread(double mset) { tau = mset / (2 * CH_PI); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;
};

CH_CLASS_VERSION(ChLinkLockScrew, 0)

}  // end namespace chrono

#endif
