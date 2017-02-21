// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================

#include "chrono/physics/ChLinkRotSpringCB.h"

namespace chrono {

// Register into the object factory, to enable run-time dynamic creation and persistence
CH_FACTORY_REGISTER(ChLinkRotSpringCB)

ChLinkRotSpringCB::ChLinkRotSpringCB() : m_torque(0), m_torque_fun(NULL) {}

ChLinkRotSpringCB::ChLinkRotSpringCB(const ChLinkRotSpringCB& other) : ChLinkMarkers(other) {
    m_torque = other.m_torque;
    m_torque_fun = other.m_torque_fun;  //// do we need a deep copy?
}

void ChLinkRotSpringCB::UpdateForces(double time) {
    // Allow the base class to update itself (possibly adding its own forces).
    ChLinkMarkers::UpdateForces(time);

    // Invoke the provided functor to evaluate torque.
    // NOTE: we implicitly assume that the kinematics are CONSISTENT with this
    // type of link!
    double angle = relAngle;
    double angle_dt = Vdot(relWvel, relAxis);

    m_torque = m_torque_fun ? (*m_torque_fun)(time, relAngle, angle_dt) : 0;

    // Add to existing torque.
    C_torque += Vmul(relAxis, m_torque);
}

void ChLinkRotSpringCB::ArchiveOUT(ChArchiveOut& marchive) {
    // version number
    marchive.VersionWrite<ChLinkRotSpringCB>();

    // serialize parent class
    ChLinkMarkers::ArchiveOUT(marchive);
}

/// Method to allow de serialization of transient data from archives.
void ChLinkRotSpringCB::ArchiveIN(ChArchiveIn& marchive) {
    // version number
    int version = marchive.VersionRead<ChLinkRotSpringCB>();

    // deserialize parent class
    ChLinkMarkers::ArchiveIN(marchive);
}

}  // end namespace chrono
