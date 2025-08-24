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

#include "chrono/solver/ChConstraintRollingNormal.h"

namespace chrono {

ChConstraintRollingNormal::ChConstraintRollingNormal()
    : rollingfriction(0), spinningfriction(0), constraint_U(NULL), constraint_V(NULL), constraint_N(NULL) {
    mode = ChConstraint::Mode::FRICTION;
}

ChConstraintRollingNormal::ChConstraintRollingNormal(const ChConstraintRollingNormal& other)
    : ChConstraintTwoTuples(other) {
    rollingfriction = other.rollingfriction;
    spinningfriction = other.spinningfriction;
    constraint_U = other.constraint_U;
    constraint_V = other.constraint_V;
    constraint_N = other.constraint_N;
}

ChConstraintRollingNormal& ChConstraintRollingNormal::operator=(const ChConstraintRollingNormal& other) {
    if (&other == this)
        return *this;

    // copy parent class data
    ChConstraintTwoTuples::operator=(other);

    rollingfriction = other.rollingfriction;
    spinningfriction = other.spinningfriction;
    constraint_U = other.constraint_U;
    constraint_V = other.constraint_V;
    constraint_N = other.constraint_N;
    return *this;
}

void ChConstraintRollingNormal::Project() {
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

}  // end namespace chrono
