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

#ifndef CH_CONSTRAINT_CONTACT_NORMAL_H
#define CH_CONSTRAINT_CONTACT_NORMAL_H

#include "chrono/solver/ChConstraintTwoTuples.h"
#include "chrono/solver/ChConstraintContactTangential.h"

namespace chrono {

/// Normal contact constraint between two objects, each represented by a tuple of ChVariables objects.
/// Used ONLY when two ChConstraintContactTangential are used to represent friction. If these two tangent constraint
/// are not used (i.e., frictionless case), use a simple ChConstraintTwo in ChConstraint::Mode::UNILATERAL mode.
class ChApi ChConstraintContactNormal : public ChConstraintTwoTuples {
  protected:
    ChConstraintContactTangential* constraint_U;  ///< U tangential component
    ChConstraintContactTangential* constraint_V;  ///< V tangential component

    double friction;  ///< friction coefficient 'f', for sqrt(Tx^2+Ty^2) < f*Nz
    double cohesion;  ///< cohesion coefficient 'c', for sqrt(Tx^2+Ty^2) < f*(Nz+c)

  public:
    ChConstraintContactNormal();

    ChConstraintContactNormal(const ChConstraintContactNormal& other);

    virtual ~ChConstraintContactNormal() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintContactNormal* Clone() const override { return new ChConstraintContactNormal(*this); }

    /// Assignment operator.
    ChConstraintContactNormal& operator=(const ChConstraintContactNormal& other);

    /// Get the friction coefficient.
    double GetFrictionCoefficient() const { return friction; }

    /// Set the friction coefficient.
    void SetFrictionCoefficient(double mcoeff) { friction = mcoeff; }

    /// Get the cohesion.
    double GetCohesion() const { return cohesion; }

    /// Set the cohesion.
    void SetCohesion(double mcoh) { cohesion = mcoh; }

    /// Get pointer to U tangential component
    ChConstraintContactTangential* GetTangentialConstraintU() const { return constraint_U; }

    /// Get pointer to V tangential component
    ChConstraintContactTangential* GetTangentialConstraintV() const { return constraint_V; }

    /// Set pointer to U tangential component
    void SetTangentialConstraintU(ChConstraintContactTangential* mconstr) { constraint_U = mconstr; }

    /// Set pointer to V tangential component
    void SetTangentialConstraintV(ChConstraintContactTangential* mconstr) { constraint_V = mconstr; }

    /// Project the value of a possible 'l_i' value of constraint reaction onto admissible set.
    /// This projection onto the friction cone will also modify the l_i values of the two tangential friction
    /// constraints.
    virtual void Project() override;
};

}  // end namespace chrono

#endif
