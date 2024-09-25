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

#ifndef CHCONSTRAINTTWOTUPLESROLLINGN_H
#define CHCONSTRAINTTWOTUPLESROLLINGN_H

#include "chrono/solver/ChConstraintTwoTuplesContactN.h"
#include "chrono/solver/ChConstraintTwoTuplesRollingT.h"

namespace chrono {

/// This is enough to use dynamic_casting<> to detect all template types
/// from ChConstraintTwoTuplesRollingN
class ChApi ChConstraintTwoTuplesRollingNall {};

/// Normal reaction between two objects, each represented by a tuple of ChVariables objects.
/// Used ONLY when also two ChConstraintTwoTuplesFrictionT objects are used to represent friction. If these two tangent
/// constraint are not used, for frictionless case, use a simple ChConstraintTwo with the ChConstraint::Mode::UNILATERAL
/// mode.
///
/// Differently from an unilateral constraint, this does not enforce projection on positive constraint, since it will be
/// up to the 'companion' ChConstraintTwoTuplesFrictionT objects to call a projection on the cone, by modifying all the
/// three components (normal, u, v) at once.
///
/// Templates Ta and Tb are of ChVariableTupleCarrier_Nvars classes.
template <class Ta, class Tb>
class ChApi ChConstraintTwoTuplesRollingN : public ChConstraintTwoTuples<Ta, Tb>,
                                            public ChConstraintTwoTuplesRollingNall {
  protected:
    float rollingfriction;   ///< the rolling friction coefficient
    float spinningfriction;  ///< the spinning friction coefficient

    ChConstraintTwoTuplesRollingT<Ta, Tb>* constraint_U;  ///< the pointer to U tangential component
    ChConstraintTwoTuplesRollingT<Ta, Tb>* constraint_V;  ///< the pointer to V tangential component
    ChConstraintTwoTuplesContactN<Ta, Tb>* constraint_N;  ///< the pointer to N  component

  public:
    ChConstraintTwoTuplesRollingN()
        : rollingfriction(0), spinningfriction(0), constraint_U(NULL), constraint_V(NULL), constraint_N(NULL) {
        this->mode = ChConstraint::Mode::FRICTION;
    }

    ChConstraintTwoTuplesRollingN(const ChConstraintTwoTuplesRollingN& other) : ChConstraintTwoTuples<Ta, Tb>(other) {
        rollingfriction = other.rollingfriction;
        spinningfriction = other.spinningfriction;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
        constraint_N = other.constraint_N;
    }

    virtual ~ChConstraintTwoTuplesRollingN() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChConstraintTwoTuplesRollingN* Clone() const override { return new ChConstraintTwoTuplesRollingN(*this); }

    /// Assignment operator: copy from other object
    ChConstraintTwoTuplesRollingN& operator=(const ChConstraintTwoTuplesRollingN& other) {
        if (&other == this)
            return *this;
        // copy parent class data
        ChConstraintTwoTuples<Ta, Tb>::operator=(other);

        rollingfriction = other.rollingfriction;
        spinningfriction = other.spinningfriction;
        constraint_U = other.constraint_U;
        constraint_V = other.constraint_V;
        constraint_N = other.constraint_N;
        return *this;
    }

    /// Get the rolling friction coefficient
    float GetRollingFrictionCoefficient() { return rollingfriction; }
    /// Set the rolling friction coefficient
    void SetRollingFrictionCoefficient(float mcoeff) { rollingfriction = mcoeff; }

    /// Get the spinning friction coefficient
    float GetSpinningFrictionCoefficient() { return spinningfriction; }
    /// Set the spinning friction coefficient
    void SetSpinningFrictionCoefficient(float mcoeff) { spinningfriction = mcoeff; }

    /// Get pointer to U tangential component
    ChConstraintTwoTuplesRollingT<Ta, Tb>* GetRollingConstraintU() { return constraint_U; }
    /// Get pointer to V tangential component
    ChConstraintTwoTuplesRollingT<Ta, Tb>* GetRollingConstraintV() { return constraint_V; }
    /// Get pointer to normal contact component
    ChConstraintTwoTuplesContactN<Ta, Tb>* GetNormalConstraint() { return constraint_N; }

    /// Set pointer to U tangential component
    void SetRollingConstraintU(ChConstraintTwoTuplesRollingT<Ta, Tb>* mconstr) { constraint_U = mconstr; }
    /// Set pointer to V tangential component
    void SetRollingConstraintV(ChConstraintTwoTuplesRollingT<Ta, Tb>* mconstr) { constraint_V = mconstr; }
    /// Set pointer to normal contact component
    void SetNormalConstraint(ChConstraintTwoTuplesContactN<Ta, Tb>* mconstr) { constraint_N = mconstr; }

    /// For iterative solvers: project the value of a possible
    /// 'l_i' value of constraint reaction onto admissible set.
    /// This projection will also modify the l_i values of the two
    /// tangential friction constraints (projection onto the friction cone,
    /// as by Anitescu-Tasora theory).
    virtual void Project() override {
        if (!constraint_U)
            return;

        if (!constraint_V)
            return;

        if (!constraint_N)
            return;

        // METHOD
        // Anitescu-Tasora projection on rolling-friction cone generator and polar cone
        // (contractive, but performs correction on three components: normal,u,v)

        double f_n = constraint_N->GetLagrangeMultiplier();
        double t_n = this->GetLagrangeMultiplier();
        double t_u = constraint_U->GetLagrangeMultiplier();
        double t_v = constraint_V->GetLagrangeMultiplier();
        double t_tang = std::sqrt(t_v * t_v + t_u * t_u);
        double t_sptang = std::fabs(t_n);  // = sqrt(t_n*t_n);

        // A. Project the spinning friction (approximate - should do cone
        //   projection stuff as in B, but spinning friction is usually very low...)

        if (spinningfriction) {
            if (t_sptang < spinningfriction * f_n) {
                // inside upper cone? keep untouched!
            } else {
                // inside lower cone? reset  normal,u,v to zero!
                if ((t_sptang < -(1.0 / spinningfriction) * f_n) || (fabs(f_n) < 10e-15)) {
                    constraint_N->SetLagrangeMultiplier(0);
                    this->SetLagrangeMultiplier(0);
                } else {
                    // remaining case: project orthogonally to generator segment of upper cone (CAN BE simplified)
                    double f_n_proj = (t_sptang * spinningfriction + f_n) / (spinningfriction * spinningfriction + 1);
                    double t_tang_proj = f_n_proj * spinningfriction;
                    double tproj_div_t = t_tang_proj / t_sptang;
                    double t_n_proj = tproj_div_t * t_n;

                    constraint_N->SetLagrangeMultiplier(f_n_proj);
                    this->SetLagrangeMultiplier(t_n_proj);
                }
            }
        }

        // B. Project the rolling friction

        // shortcut
        if (!rollingfriction) {
            constraint_U->SetLagrangeMultiplier(0);
            constraint_V->SetLagrangeMultiplier(0);
            if (f_n < 0)
                constraint_N->SetLagrangeMultiplier(0);
            return;
        }

        // inside upper cone? keep untouched!
        if (t_tang < rollingfriction * f_n)
            return;

        // inside lower cone? reset  normal,u,v to zero!
        if ((t_tang < -(1.0 / rollingfriction) * f_n) || (fabs(f_n) < 10e-15)) {
            double f_n_proj = 0;
            double t_u_proj = 0;
            double t_v_proj = 0;

            constraint_N->SetLagrangeMultiplier(f_n_proj);
            constraint_U->SetLagrangeMultiplier(t_u_proj);
            constraint_V->SetLagrangeMultiplier(t_v_proj);

            return;
        }

        // remaining case: project orthogonally to generator segment of upper cone
        double f_n_proj = (t_tang * rollingfriction + f_n) / (rollingfriction * rollingfriction + 1);
        double t_tang_proj = f_n_proj * rollingfriction;
        double tproj_div_t = t_tang_proj / t_tang;
        double t_u_proj = tproj_div_t * t_u;
        double t_v_proj = tproj_div_t * t_v;

        constraint_N->SetLagrangeMultiplier(f_n_proj);
        constraint_U->SetLagrangeMultiplier(t_u_proj);
        constraint_V->SetLagrangeMultiplier(t_v_proj);
    }
};

}  // end namespace chrono

#endif
