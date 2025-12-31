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

#ifndef CH_SHAFTS_TORQUE_H
#define CH_SHAFTS_TORQUE_H

#include "chrono/physics/ChShaftsCouple.h"
#include "chrono/solver/ChConstraintTwoGeneric.h"

namespace chrono {

/// Base class for all classes defining a torque between two one-degree-of-freedom parts.
/// Examples include torsional dampers, torsional springs, electric engines, etc.
class ChApi ChShaftsTorque : public ChShaftsCouple {
  public:
    ChShaftsTorque();
    ChShaftsTorque(const ChShaftsTorque& other);
    virtual ~ChShaftsTorque() {}

    /// Number of scalar constraints
    virtual unsigned int GetNumConstraintsBilateral() override { return 0; }

    /// Get the reaction torque exchanged between the two shafts, considered as applied to the 1st axis.
    virtual double GetReaction1() const override { return torque; }

    /// Get the reaction torque exchanged between the two shafts, considered as applied to the 2nd axis.
    virtual double GetReaction2() const override { return -torque; }

    /// Calculate applied torque.
    /// In most cases, this is the only function a derived class must implement. It will be called at each Update().
    virtual double ComputeTorque() = 0;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  protected:
    double torque;  ///< current value of torque

    virtual void Update(double time, bool update_assets) override;

    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;

    void VariablesFbLoadForces(double factor = 1) override;
};

CH_CLASS_VERSION(ChShaftsTorque, 0)

}  // end namespace chrono

#endif
