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

#ifndef CHCONSTRAINTTWOTUPLESCONTACTN_H
#define CHCONSTRAINTTWOTUPLESCONTACTN_H

#include "chrono/solver/ChConstraintTwoTuplesFrictionT.h"

namespace chrono {

class ChApi ChConstraintTwoTuplesContactNall {
  public:
    /// Get the friction coefficient.
    double GetFrictionCoefficient() const { return friction; }

    /// Set the friction coefficient.
    void SetFrictionCoefficient(double mcoeff) { friction = mcoeff; }

    /// Get the cohesion.
    double GetCohesion() const { return cohesion; }

    /// Set the cohesion.
    void SetCohesion(double mcoh) { cohesion = mcoh; }

  protected:
    double friction;  ///< friction coefficient 'f', for sqrt(Tx^2+Ty^2)<f*Nz
    double cohesion;  ///< cohesion 'c', non-negative, for sqrt(Tx^2+Ty^2)<f*(Nz+c)
};

/// Normal reaction between two objects, each represented by a tuple of ChVariables objects.
/// Used ONLY when also two ChConstraintTwoTuplesFrictionT objects are used to represent friction. If these two tangent
/// constraint are not used, for frictionless case, use a simple ChConstraintTwo with the ChConstraint::Mode::UNILATERAL
/// mode.
///
/// Differently from an unilateral constraint, this does not enforce projection on positive constraint, since it will be
/// up to the 'companion' ChConstraintTwoTuplesFrictionT objects to call a projection on the cone, by modifying all the
/// three components (normal, u, v) at once.
///
/// Templates Ta and Tb are of ChVariableTupleCarrier_Nvars classes
template <class Ta, class Tb>
class ChApi ChConstraintTwoTuplesContactN : public ChConstraintTwoTuples<Ta, Tb>,
                                            public ChConstraintTwoTuplesContactNall {
  protected:
    ChConstraintTwoTuplesFrictionT<Ta, Tb>* constraint_U;  ///< U tangential component
    ChConstraintTwoTuplesFrictionT<Ta, Tb>* constraint_V;  ///< V tangential component

  public:
    ChConstraintTwoTuplesContactN() {
        this->mode = ChConstraint::Mode::FRICTION;
        friction = 0.0;
        cohesion = 0.0;
        constraint_U = constraint_V = 0;
    }

    ChConstraintTwoTuplesContactN(const ChConstraintTwoTuplesContactN& other) : ChConstraintTwoTuples<Ta, Tb>(other) {
        friction = other.friction;
        cohesion = other.cohesion;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
    }

    virtual ~ChConstraintTwoTuplesContactN() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoTuplesContactN* Clone() const override { return new ChConstraintTwoTuplesContactN(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoTuplesContactN& operator=(const ChConstraintTwoTuplesContactN& other) {
        if (&other == this)
            return *this;
        // copy parent class data
        ChConstraintTwoTuples<Ta, Tb>::operator=(other);

        friction = other.friction;
        cohesion = other.cohesion;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
        return *this;
    }

    /// Get pointer to U tangential component
    ChConstraintTwoTuplesFrictionT<Ta, Tb>* GetTangentialConstraintU() const { return constraint_U; }
    /// Get pointer to V tangential component
    ChConstraintTwoTuplesFrictionT<Ta, Tb>* GetTangentialConstraintV() const { return constraint_V; }

    /// Set pointer to U tangential component
    void SetTangentialConstraintU(ChConstraintTwoTuplesFrictionT<Ta, Tb>* mconstr) { constraint_U = mconstr; }
    /// Set pointer to V tangential component
    void SetTangentialConstraintV(ChConstraintTwoTuplesFrictionT<Ta, Tb>* mconstr) { constraint_V = mconstr; }

    /// Project the value of a possible 'l_i' value of constraint reaction onto admissible set.
    /// This will also modify the l_i values of the two tangential friction constraints
    /// (projection onto the friction cone, as by Anitescu-Tasora theory).
    virtual void Project() override {
        if (!constraint_U || !constraint_V)
            return;

        // Anitescu-Tasora projection on cone generator and polar cone
        // (contractive, but performs correction on three components: normal,u,v)

        double f_n = this->l_i + this->cohesion;

        // no friction? project to axis of upper cone
        if (friction == 0) {
            constraint_U->SetLagrangeMultiplier(0);
            constraint_V->SetLagrangeMultiplier(0);
            if (f_n < 0)
                this->SetLagrangeMultiplier(0);
            return;
        }

        double f_u = constraint_U->GetLagrangeMultiplier();
        double f_v = constraint_V->GetLagrangeMultiplier();

        double mu2 = friction * friction;
        double f_n2 = f_n * f_n;
        double f_t2 = (f_v * f_v + f_u * f_u);

        // inside lower cone or close to origin? reset normal, u, v to zero!
        if ((f_n <= 0 && f_t2 < f_n2 / mu2) || (f_n < 1e-14 && f_n > -1e-14)) {
            this->SetLagrangeMultiplier(0);
            constraint_U->SetLagrangeMultiplier(0);
            constraint_V->SetLagrangeMultiplier(0);
            return;
        }

        // inside upper cone? keep untouched!
        if (f_t2 < f_n2 * mu2)
            return;

        // project orthogonally to generator segment of upper cone
        double f_t = std::sqrt(f_t2);
        double f_n_proj = (f_t * friction + f_n) / (mu2 + 1);
        double f_t_proj = f_n_proj * friction;
        double tproj_div_t = f_t_proj / f_t;
        double f_u_proj = tproj_div_t * f_u;
        double f_v_proj = tproj_div_t * f_v;

        this->SetLagrangeMultiplier(f_n_proj - this->cohesion);
        constraint_U->SetLagrangeMultiplier(f_u_proj);
        constraint_V->SetLagrangeMultiplier(f_v_proj);
    }
};

}  // end namespace chrono

#endif
