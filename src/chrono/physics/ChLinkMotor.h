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

#ifndef CHLINKMOTOR_H
#define CHLINKMOTOR_H

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/motion_functions/ChFunction.h"

namespace chrono {

/// Base class for all "motor" constraints between
/// two frames on two bodies.
/// Look for children classes for specialized behaviors,
/// for example chrono::ChLinkMotorRotationAngle.

class ChApi ChLinkMotor : public ChLinkMateGeneric {
  public:
    ChLinkMotor();
    ChLinkMotor(const ChLinkMotor& other);
    virtual ~ChLinkMotor();

    /// Set the actuation function of time F(t).
    /// The return value of this function has different meanings in various derived classes
    /// and can represent a position, angle, linear speed, angular speed, force, or torque.
    /// If controlling a position-level quantity (position or angle), this function must be
    /// C0 continuous (ideally C1 continuous to prevent spikes in accelerations).
    /// If controlling a velocity-level quantity (linear on angular speed), this function 
    /// should ideally be C0 continuous to prevent acceleration spikes.
    void SetMotorFunction(const std::shared_ptr<ChFunction> function) { m_func = function; }

    /// Get the actuation function F(t).
    std::shared_ptr<ChFunction> GetMotorFunction() const { return m_func; }

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotor* Clone() const override { return new ChLinkMotor(*this); }

    /// Update state of the LinkMotor.
    virtual void Update(double mytime, bool update_assets) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  protected:
    std::shared_ptr<ChFunction> m_func;
};

CH_CLASS_VERSION(ChLinkMotor, 0)

}  // end namespace chrono

#endif
