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

#include "chrono/physics/ChLinkForce.h"

namespace chrono {

ChLinkForce::ChLinkForce() : m_active(false), m_F(0), m_K(0), m_R(0) {
    // Default: no modulation
    m_F_modul = std::make_shared<ChFunction_Const>(1);
    m_K_modul = std::make_shared<ChFunction_Const>(1);
    m_R_modul = std::make_shared<ChFunction_Const>(1);
}

ChLinkForce::ChLinkForce(const ChLinkForce& other) {
    m_active = other.m_active;

    m_F = other.m_F;
    m_K = other.m_K;
    m_R = other.m_R;

    m_F_modul = std::shared_ptr<ChFunction>(other.m_F_modul->Clone());
    m_K_modul = std::shared_ptr<ChFunction>(other.m_K_modul->Clone());
    m_R_modul = std::shared_ptr<ChFunction>(other.m_R_modul->Clone());
}

double ChLinkForce::GetKcurrent(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return m_K * m_K_modul->Get_y(x);
}

double ChLinkForce::GetRcurrent(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return m_R * m_R_modul->Get_y(x);
}

double ChLinkForce::GetFcurrent(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return m_F * m_F_modul->Get_y(t);
}

double ChLinkForce::GetForce(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return m_F * m_F_modul->Get_y(t) - (m_K * m_K_modul->Get_y(x)) * x - (m_R * m_R_modul->Get_y(x)) * x_dt;
}

void ChLinkForce::ArchiveOUT(ChArchiveOut& marchive) {
    // class version number
    marchive.VersionWrite<ChLinkForce>();

    // stream out all member data
    marchive << CHNVP(m_active);
    marchive << CHNVP(m_F);
    marchive << CHNVP(m_F_modul);
    marchive << CHNVP(m_K);
    marchive << CHNVP(m_K_modul);
    marchive << CHNVP(m_R);
    marchive << CHNVP(m_R_modul);
}

void ChLinkForce::ArchiveIN(ChArchiveIn& marchive) {
    // class version number
    int version = marchive.VersionRead<ChLinkForce>();

    // stream in all member data
    marchive >> CHNVP(m_active);
    marchive >> CHNVP(m_F);
    marchive >> CHNVP(m_F_modul);
    marchive >> CHNVP(m_K);
    marchive >> CHNVP(m_K_modul);
    marchive >> CHNVP(m_R);
    marchive >> CHNVP(m_R_modul);
}

}  // end namespace chrono
