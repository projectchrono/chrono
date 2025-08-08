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

#ifndef CH_CONSTRAINT_ROLLING_NORMAL_H
#define CH_CONSTRAINT_ROLLING_NORMAL_H

#include "chrono/solver/ChConstraintTwoTuples.h"
#include "chrono/solver/ChConstraintContactNormal.h"
#include "chrono/solver/ChConstraintRollingTangential.h"

namespace chrono {

/// Normal rolling contact constraint between two objects, each represented by a tuple of ChVariables objects.
class ChApi ChConstraintRollingNormal : public ChConstraintTwoTuples {
  protected:
    float rollingfriction;   ///< the rolling friction coefficient
    float spinningfriction;  ///< the spinning friction coefficient

    ChConstraintContactNormal* constraint_N;      ///< the pointer to N  component
    ChConstraintRollingTangential* constraint_U;  ///< the pointer to U tangential component
    ChConstraintRollingTangential* constraint_V;  ///< the pointer to V tangential component

  public:
    ChConstraintRollingNormal();

    ChConstraintRollingNormal(const ChConstraintRollingNormal& other);

    virtual ~ChConstraintRollingNormal() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintRollingNormal* Clone() const override { return new ChConstraintRollingNormal(*this); }

    /// Assignment operator.
    ChConstraintRollingNormal& operator=(const ChConstraintRollingNormal& other);

    /// Get the rolling friction coefficient
    float GetRollingFrictionCoefficient() { return rollingfriction; }
    /// Set the rolling friction coefficient
    void SetRollingFrictionCoefficient(float mcoeff) { rollingfriction = mcoeff; }

    /// Get the spinning friction coefficient
    float GetSpinningFrictionCoefficient() { return spinningfriction; }
    /// Set the spinning friction coefficient
    void SetSpinningFrictionCoefficient(float mcoeff) { spinningfriction = mcoeff; }

    /// Get pointer to U tangential component
    ChConstraintRollingTangential* GetRollingConstraintU() { return constraint_U; }

    /// Get pointer to V tangential component
    ChConstraintRollingTangential* GetRollingConstraintV() { return constraint_V; }

    /// Get pointer to normal contact component
    ChConstraintContactNormal* GetNormalConstraint() { return constraint_N; }

    /// Set pointer to U tangential component
    void SetRollingConstraintU(ChConstraintRollingTangential* mconstr) { constraint_U = mconstr; }

    /// Set pointer to V tangential component
    void SetRollingConstraintV(ChConstraintRollingTangential* mconstr) { constraint_V = mconstr; }

    /// Set pointer to normal contact component
    void SetNormalConstraint(ChConstraintContactNormal* mconstr) { constraint_N = mconstr; }

    /// Project the value of a possible 'l_i' value of constraint reaction onto admissible set.
    /// This projection onto the friction cone will also modify the l_i values of the two tangential friction
    /// constraints.
    virtual void Project() override;
};

}  // end namespace chrono

#endif
