// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Ian Rust
// =============================================================================
//
// Template for a fifth wheel chassis connector.  This is a passive connector,
// modeled with a universal joint.
//
// =============================================================================

#include "chrono_vehicle/chassis/ChChassisConnectorFifthWheel.h"

namespace chrono {
namespace vehicle {

ChChassisConnectorFifthWheel::ChChassisConnectorFifthWheel(const std::string& name) : ChChassisConnector(name) {}

ChChassisConnectorFifthWheel::~ChChassisConnectorFifthWheel() {
    if (!IsInitialized())
        return;

    auto sys = m_joint->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_joint);
}

void ChChassisConnectorFifthWheel::Initialize(std::shared_ptr<ChChassis> front, std::shared_ptr<ChChassisRear> rear) {
    // Express the connector reference frame in the absolute coordinate system
    // Rotate universal joint frames to allow pitch and yaw relative DOFs
    ChFrame<> to_abs(rear->GetLocalPosFrontConnector(), Q_ROTATE_Z_TO_X);
    to_abs.ConcatenatePreTransformation(rear->GetBody()->GetFrameRefToAbs());

    // Create and initialize universal joint
    m_joint = chrono_types::make_shared<ChLinkUniversal>();
    m_joint->SetName(m_name + " joint");
    m_joint->Initialize(front->GetBody(), rear->GetBody(), to_abs);
    rear->GetBody()->GetSystem()->AddLink(m_joint);

    ChPart::Initialize();
}

void ChChassisConnectorFifthWheel::PopulateComponentList() {
    m_joints.push_back(m_joint);
}

}  // end namespace vehicle
}  // end namespace chrono
