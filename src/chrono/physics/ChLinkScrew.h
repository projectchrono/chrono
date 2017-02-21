// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
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

class ChApi ChLinkScrew : public ChLinkLock {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChLinkScrew)

  protected:
    double tau;  ///< transmission coeff.

  public:
    ChLinkScrew();
    ChLinkScrew(const ChLinkScrew& other);
    virtual ~ChLinkScrew() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkScrew* Clone() const override { return new ChLinkScrew(*this); }

    // Inherit the link-lock computations like it were a
    // normal "revolute" joint, but then modifies the Z-lock parts of C,
    // Cdt, Cdtdt, [Cq] etc., in order to have z = tau * alpha.
    virtual void UpdateState() override;

    double Get_tau() const { return tau; };
    void Set_tau(double mset) { tau = mset; }
    double Get_thread() const { return tau * (2 * CH_C_PI); };
    void Set_thread(double mset) { tau = mset / (2 * CH_C_PI); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkScrew,0)

}  // end namespace chrono

#endif
