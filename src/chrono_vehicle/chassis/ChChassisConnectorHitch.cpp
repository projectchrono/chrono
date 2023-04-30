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
//
// Template for a hitch chassis connector.  This is a passive connector,
// modeled with a spherical joint.
//
// =============================================================================

#include "chrono_vehicle/chassis/ChChassisConnectorHitch.h"

namespace chrono {
namespace vehicle {

ChChassisConnectorHitch::ChChassisConnectorHitch(const std::string& name) : ChChassisConnector(name) {}

ChChassisConnectorHitch::~ChChassisConnectorHitch() {
    auto sys = m_joint->GetSystem();
    if (sys) {
        sys->Remove(m_joint);
    }
}

void ChChassisConnectorHitch::Initialize(std::shared_ptr<ChChassis> front, std::shared_ptr<ChChassisRear> rear) {
    ChChassisConnector::Initialize(front, rear);

    // Express the connector reference frame in the absolute coordinate system
    ChFrame<> to_abs(rear->GetLocalPosFrontConnector());
    to_abs.ConcatenatePreTransformation(rear->GetBody()->GetFrame_REF_to_abs());

    // Create the revolute joint connection
    m_joint = chrono_types::make_shared<ChLinkLockSpherical>();
    m_joint->SetNameString(m_name + " joint");
    m_joint->Initialize(front->GetBody(), rear->GetBody(), ChCoordsys<>(to_abs.GetPos(), QUNIT));
    rear->GetBody()->GetSystem()->AddLink(m_joint);
}

}  // end namespace vehicle
}  // end namespace chrono
