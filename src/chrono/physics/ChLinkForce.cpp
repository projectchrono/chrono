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
    m_F_modul = chrono_types::make_shared<ChFunctionConst>(1);
    m_K_modul = chrono_types::make_shared<ChFunctionConst>(1);
    m_R_modul = chrono_types::make_shared<ChFunctionConst>(1);
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

double ChLinkForce::GetCurrentSpringCoefficient(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return m_K * m_K_modul->GetVal(x);
}

double ChLinkForce::GetCurrentDampingCoefficient(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return m_R * m_R_modul->GetVal(x);
}

double ChLinkForce::GetCurrentActuatorForceTorque(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return m_F * m_F_modul->GetVal(t);
}

double ChLinkForce::GetForceTorque(double x, double x_dt, double t) const {
    if (!m_active)
        return 0;
    return GetCurrentActuatorForceTorque(x, x_dt, t) - GetCurrentSpringCoefficient(x, x_dt, t) * x -
           GetCurrentDampingCoefficient(x, x_dt, t) * x_dt;
}

void ChLinkForce::ArchiveOut(ChArchiveOut& archive_out) {
    // class version number
    archive_out.VersionWrite<ChLinkForce>();

    // stream out all member data
    archive_out << CHNVP(m_active);
    archive_out << CHNVP(m_F);
    archive_out << CHNVP(m_F_modul);
    archive_out << CHNVP(m_K);
    archive_out << CHNVP(m_K_modul);
    archive_out << CHNVP(m_R);
    archive_out << CHNVP(m_R_modul);
}

void ChLinkForce::ArchiveIn(ChArchiveIn& archive_in) {
    // class version number
    /*int version =*/archive_in.VersionRead<ChLinkForce>();

    // stream in all member data
    archive_in >> CHNVP(m_active);
    archive_in >> CHNVP(m_F);
    archive_in >> CHNVP(m_F_modul);
    archive_in >> CHNVP(m_K);
    archive_in >> CHNVP(m_K_modul);
    archive_in >> CHNVP(m_R);
    archive_in >> CHNVP(m_R_modul);
}

}  // end namespace chrono
