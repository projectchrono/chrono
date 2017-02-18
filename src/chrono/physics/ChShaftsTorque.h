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

#ifndef CHSHAFTSTORQUE_H
#define CHSHAFTSTORQUE_H

#include "chrono/physics/ChShaftsTorqueBase.h"

namespace chrono {

/// Class for defining a user-defined torque between two one-degree-of-freedom parts;
/// i.e., shafts that can be used to build 1D models of powertrains. This is more
/// efficient than simulating power trains modeled with full 3D ChBody objects.
/// Two shaftsa are needed, because one gets the torque, and the other is the
/// 'reference' that gets the negative torque as a reaction. For instance, a
/// thermal engine applies a torque T to a crankshaft, but also applies -T to the
/// engine block!
/// Note that another way of imposing a torque is to do just
///       my_shaftA->SetAppliedTorque(6);
/// but in such case is an 'absolute' torque does not create a reaction.

class ChApi ChShaftsTorque : public ChShaftsTorqueBase {

    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChShaftsTorque)

  public:
    ChShaftsTorque() {}
    ChShaftsTorque(const ChShaftsTorque& other);
    ~ChShaftsTorque() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsTorque* Clone() const override { return new ChShaftsTorque(*this); }

    /// Set the imposed torque between the two shafts
    void SetTorque(double mt) { torque = mt; }
    /// Get the imposed torque between the two shafts
    double GetTorque() const { return torque; }

    /// (does nothing, just eaves the last user defined torque)
    virtual double ComputeTorque() override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChShaftsTorque,0)

}  // end namespace chrono

#endif
