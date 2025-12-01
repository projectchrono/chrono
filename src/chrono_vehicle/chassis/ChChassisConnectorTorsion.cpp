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
// Template for a torsion chassis connector.  This is a passive connector,
// modeled with a revolute joint (aligned with the vehicle's longitudinal axis)
// and a rotational spring-damper.
//
// =============================================================================

#include "chrono_vehicle/chassis/ChChassisConnectorTorsion.h"

namespace chrono {
namespace vehicle {

ChChassisConnectorTorsion::ChChassisConnectorTorsion(const std::string& name) : ChChassisConnector(name) {}

ChChassisConnectorTorsion::~ChChassisConnectorTorsion() {
    if (!IsInitialized())
        return;

    auto sys = m_joint->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_joint);
    sys->Remove(m_spring);
}

void ChChassisConnectorTorsion::Initialize(std::shared_ptr<ChChassis> front, std::shared_ptr<ChChassisRear> rear) {
    // Express the connector reference frame in the absolute coordinate system
    ChFrame<> to_abs(rear->GetLocalPosFrontConnector());
    to_abs.ConcatenatePreTransformation(rear->GetBody()->GetFrameRefToAbs());

    ChQuaternion<> chassisRot = rear->GetBody()->GetFrameRefToAbs().GetRot();
    ChFrame<> rev_frame(to_abs.GetPos(), chassisRot * QuatFromAngleY(CH_PI / 2.0));

    // Create the revolute joint connection
    m_joint = chrono_types::make_shared<ChLinkLockRevolute>();
    m_joint->SetName(m_name + " joint");
    m_joint->Initialize(front->GetBody(), rear->GetBody(), rev_frame);
    rear->GetBody()->GetSystem()->AddLink(m_joint);

    // Create the rotational spring-damper (as a model of chassis torsional stiffness)
    m_spring = chrono_types::make_shared<ChLinkRSDA>();
    m_spring->SetName(m_name + " torsionSpring");
    m_spring->Initialize(front->GetBody(), rear->GetBody(), rev_frame);
    double K = GetTorsionStiffness();
    double C = K / 100;  // damping should not be zero
    auto cb = chrono_types::make_shared<utils::LinearSpringDamperTorque>(K, C, 0);
    m_spring->RegisterTorqueFunctor(cb);
    rear->GetBody()->GetSystem()->AddLink(m_spring);

    ChPart::Initialize();
}

void ChChassisConnectorTorsion::PopulateComponentList() {
    m_joints.push_back(m_joint);
    m_rsdas.push_back(m_spring);
}

}  // end namespace vehicle
}  // end namespace chrono
