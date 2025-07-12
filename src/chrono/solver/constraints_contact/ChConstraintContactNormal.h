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

#include "chrono/solver/constraints_contact/ChConstraintTwoTuples.h"
#include "chrono/solver/constraints_contact/ChConstraintContactTangential.h"

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
    ChConstraintContactNormal() {
        this->mode = ChConstraint::Mode::FRICTION;
        friction = 0.0;
        cohesion = 0.0;
        constraint_U = constraint_V = 0;
    }

    ChConstraintContactNormal(const ChConstraintContactNormal& other) : ChConstraintTwoTuples(other) {
        friction = other.friction;
        cohesion = other.cohesion;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
    }

    virtual ~ChConstraintContactNormal() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintContactNormal* Clone() const override { return new ChConstraintContactNormal(*this); }

    /// Assignment operator.
    ChConstraintContactNormal& operator=(const ChConstraintContactNormal& other) {
        if (&other == this)
            return *this;

        // copy parent class data
        ChConstraintTwoTuples::operator=(other);

        friction = other.friction;
        cohesion = other.cohesion;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
        
        return *this;
    }
    
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
