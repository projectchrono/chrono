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


#include "chrono/physics/ChLinkMate.h"

namespace chrono {

/// Base class for all "motor" constraints between
/// two frames on two bodies. 
/// Look for children classes for specialized behaviors, 
/// for example chrono::ChLinkMotorRotationAngle.

class ChApi ChLinkMotor : public ChLinkMateGeneric {

  public:
    ChLinkMotor() {}
    ChLinkMotor(const ChLinkMotor& other) : ChLinkMateGeneric(other) {}
    virtual ~ChLinkMotor() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkMotor* Clone() const override { return new ChLinkMotor(*this); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;
};

CH_CLASS_VERSION(ChLinkMotor,0)




}  // end namespace chrono

#endif
