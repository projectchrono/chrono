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

#ifndef CH_SHAFTS_APPLIED_TORQUE_H
#define CH_SHAFTS_APPLIED_TORQUE_H

#include "chrono/physics/ChShaftsTorque.h"

namespace chrono {

/// Class for defining a user-defined torque between two one-degree-of-freedom parts.
/// Two shafts are needed, because one gets the torque, and the other is the 'reference' that gets the negative torque
/// as a reaction. For instance, a thermal engine applies a torque T to a crankshaft, but also applies -T to the engine
/// block. Note that another way of imposing a torque is to do just
///       my_shaftA->SetAppliedLoad(6);
/// but in such case, this is an 'absolute' torque does not create a reaction.
class ChApi ChShaftsAppliedTorque : public ChShaftsTorque {
  public:
    ChShaftsAppliedTorque() {}
    ChShaftsAppliedTorque(const ChShaftsAppliedTorque& other);
    ~ChShaftsAppliedTorque() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChShaftsAppliedTorque* Clone() const override { return new ChShaftsAppliedTorque(*this); }

    /// Set the imposed torque between the two shafts.
    void SetTorque(double mt) { torque = mt; }

    /// Get the imposed torque between the two shafts.
    double GetTorque() const { return torque; }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    // Does nothing, just returns the last user defined torque.
    virtual double ComputeTorque() override;
};

CH_CLASS_VERSION(ChShaftsAppliedTorque, 0)

}  // end namespace chrono

#endif
