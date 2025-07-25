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

#ifndef CH_CONSTRAINT_ROLLING_TANGENTIAL_H
#define CH_CONSTRAINT_ROLLING_TANGENTIAL_H

#include "chrono/solver/ChConstraintTwoTuples.h"

namespace chrono {

/// Rolling friction constraint between two objects, each represented by a tuple of ChVariables objects.
/// This constraint cannot be used alone, but only together with a ChConstraintRollingNormal.
class ChApi ChConstraintRollingTangential : public ChConstraintTwoTuples {
  public:
    ChConstraintRollingTangential();

    ChConstraintRollingTangential(const ChConstraintRollingTangential& other);

    virtual ~ChConstraintRollingTangential() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintRollingTangential* Clone() const override { return new ChConstraintRollingTangential(*this); }

    /// Assignment operator.
    ChConstraintRollingTangential& operator=(const ChConstraintRollingTangential& other);

    /// Indicate whether or not this constraint is linear.
    virtual bool IsLinear() const override { return false; }

    /// The constraint is satisfied?
    virtual double Violation(double mc_i) override { return 0.0; }
};

}  // end namespace chrono

#endif
