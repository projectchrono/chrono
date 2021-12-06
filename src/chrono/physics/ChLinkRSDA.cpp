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
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkRSDA.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkRSDA)

ChLinkRSDA::ChLinkRSDA() : m_k(0), m_r(0), m_t(0), m_torque_fun(nullptr), m_torque(0) {}

ChLinkRSDA::ChLinkRSDA(const ChLinkRSDA& other) : ChLinkMarkers(other) {
    m_torque = other.m_torque;
    m_torque_fun = other.m_torque_fun;  //// do we need a deep copy?
}

void ChLinkRSDA::UpdateForces(double time) {
    // Allow the base class to update itself (possibly adding its own forces).
    ChLinkMarkers::UpdateForces(time);

    // Calculate torque in the spring direction and convert to 3-D torque.
    double relAngle_dt = Vdot(relWvel, relAxis);
    if (m_torque_fun) {
        m_torque = m_torque_fun->evaluate(time, relAngle, relAngle_dt, this);
    } else {
        m_torque = m_t - m_k * relAngle - m_r * relAngle_dt;
    }
    // Add to existing torque.
    C_torque += Vmul(relAxis, m_torque);
}

void ChLinkRSDA::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkRSDA>();

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);
}

void ChLinkRSDA::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    /*int version =*/ marchive.VersionRead<ChLinkRSDA>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);
}

}  // end namespace chrono
