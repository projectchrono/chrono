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

#ifndef CHSHAFTSTORSIONSPRING_H
#define CHSHAFTSTORSIONSPRING_H

#include "chrono/physics/ChShaftsTorqueBase.h"

namespace chrono {

/// Class for defining a torsional spring-damper between between two 1D parts;
/// i.e., shafts that can be used to build 1D models of powertrains. This is
/// more efficient than simulating power trains modeled with full 3D ChBody
/// objects.

class ChApi ChShaftsTorsionSpring : public ChShaftsTorqueBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsTorsionSpring)

  private:
    double stiffness;
    double damping;

  public:
    ChShaftsTorsionSpring();
    ChShaftsTorsionSpring(const ChShaftsTorsionSpring& other);
    ~ChShaftsTorsionSpring() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsTorsionSpring* Clone() const override { return new ChShaftsTorsionSpring(*this); }

    /// Set the torsional stiffness between the two shafts
    void SetTorsionalStiffness(double mt) { stiffness = mt; }
    /// Get the torsional stiffness between the two shafts
    double GetTorsionalStiffness() const { return stiffness; }

    /// Set the torsional damping between the two shafts
    void SetTorsionalDamping(double mt) { damping = mt; }
    /// Get the torsional damping between the two shafts
    double GetTorsionalDamping() const { return damping; }

    /// This is the function that actually contains the
    /// formula for computing T=T(rot,vel,time,etc)
    virtual double ComputeTorque() override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsTorsionSpring,0)

}  // end namespace chrono

#endif
