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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHLINKMOTORLINEARFORCE_H
#define CHLINKMOTORLINEARFORCE_H

#include "chrono/physics/ChLinkMotorLinear.h"

namespace chrono {

/// A linear motor that applies a force between two frames on two bodies along Z axis.
/// Differently from the ChLinkMotorLinearPosition and ChLinkMotorLinearSpeed, this does not enforce precise motion via
/// constraint. Application examples:
/// - mimic a PID controlled system with some feedback (user-defined)
/// - force that is updated by a cosimulation
/// - force from a human-in-the-loop setpoint
/// Use SetForceFunction() to change to other force function (default zero force).
class ChApi ChLinkMotorLinearForce : public ChLinkMotorLinear {
  public:
    ChLinkMotorLinearForce();
    ChLinkMotorLinearForce(const ChLinkMotorLinearForce& other);
    virtual ~ChLinkMotorLinearForce();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotorLinearForce* Clone() const override { return new ChLinkMotorLinearForce(*this); }

    /// Set the force function of time F(t).
    void SetForceFunction(const std::shared_ptr<ChFunction> function) { SetMotorFunction(function); }

    /// Get the force function F(t).
    std::shared_ptr<ChFunction> GetForceFunction() const { return GetMotorFunction(); }

    /// Get the current actuator reaction force.
    virtual double GetMotorForce() const override { return m_func->GetVal(GetChTime()); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in) override;

  private:
    virtual void Update(double time, bool update_assets) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;
};

CH_CLASS_VERSION(ChLinkMotorLinearForce, 0)

}  // end namespace chrono

#endif
