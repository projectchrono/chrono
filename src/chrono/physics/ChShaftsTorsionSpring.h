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

#ifndef CHSHAFTSTORSIONSPRING_H
#define CHSHAFTSTORSIONSPRING_H

#include "chrono/physics/ChShaftsTorque.h"

namespace chrono {

/// Class for defining a torsional spring-damper between two 1D parts.
class ChApi ChShaftsTorsionSpring : public ChShaftsTorque {
  public:
    ChShaftsTorsionSpring();
    ChShaftsTorsionSpring(const ChShaftsTorsionSpring& other);
    ~ChShaftsTorsionSpring() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsTorsionSpring* Clone() const override { return new ChShaftsTorsionSpring(*this); }

    /// Set the torsional stiffness between the two shafts.
    void SetTorsionalStiffness(double mt) { stiffness = mt; }

    /// Get the torsional stiffness between the two shafts.
    double GetTorsionalStiffness() const { return stiffness; }

    /// Set the torsional damping between the two shafts.
    void SetTorsionalDamping(double mt) { damping = mt; }

    /// Get the torsional damping between the two shafts.
    double GetTorsionalDamping() const { return damping; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    double stiffness;
    double damping;

    /// Compute T=T(rot,vel,time,etc).
    virtual double ComputeTorque() override;
};

CH_CLASS_VERSION(ChShaftsTorsionSpring, 0)

}  // end namespace chrono

#endif
