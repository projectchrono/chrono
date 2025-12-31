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

#include "chrono/solver/ChConstraintContactNormal.h"

namespace chrono {

ChConstraintContactNormal::ChConstraintContactNormal() {
    mode = ChConstraint::Mode::FRICTION;
    friction = 0.0;
    cohesion = 0.0;
    constraint_U = constraint_V = 0;
}

ChConstraintContactNormal::ChConstraintContactNormal(const ChConstraintContactNormal& other)
    : ChConstraintTwoTuples(other) {
    friction = other.friction;
    cohesion = other.cohesion;
    constraint_U = other.constraint_U;
    constraint_V = other.constraint_V;
}

ChConstraintContactNormal& ChConstraintContactNormal::operator=(const ChConstraintContactNormal& other) {
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

void ChConstraintContactNormal::Project() {
    if (!constraint_U || !constraint_V)
        return;

    // Anitescu-Tasora projection on cone generator and polar cone
    // (contractive, but performs correction on three components: normal,u,v)

    double f_n = l_i + cohesion;

    // no friction? project to axis of upper cone
    if (friction == 0) {
        constraint_U->SetLagrangeMultiplier(0);
        constraint_V->SetLagrangeMultiplier(0);
        if (f_n < 0)
            SetLagrangeMultiplier(0);
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

    this->SetLagrangeMultiplier(f_n_proj - cohesion);
    constraint_U->SetLagrangeMultiplier(f_u_proj);
    constraint_V->SetLagrangeMultiplier(f_v_proj);
}

}  // end namespace chrono
