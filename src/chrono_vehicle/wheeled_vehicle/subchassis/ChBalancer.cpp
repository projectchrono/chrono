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
// Base class for a sub-chassis system for wheeled vehicles.
//
// =============================================================================

#include "chrono/physics/ChSystem.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"

namespace chrono {
namespace vehicle {

ChBalancer::ChBalancer(const std::string& name) : ChSubchassis(name) {}

// -----------------------------------------------------------------------------

void ChBalancer::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    m_location = location;

    // Express the subchassis reference frame in the absolute coordinate system
    ChFrame<> to_abs(location);
    to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Transform all hardpoints to absolute frame
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = GetLocation(static_cast<PointId>(i));
        m_pointsL[i] = to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = to_abs.TransformLocalToParent(rel_pos);
    }

    // Orientation of revolute joints
    ChQuaternion<> joint_rot = chassisRot * Q_from_AngX(CH_C_PI_2);

    // Create left side beam body
    m_beam[LEFT] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_beam[LEFT]->SetNameString(m_name + "_balancer_L");
    m_beam[LEFT]->SetPos(m_pointsL[BEAM]);
    m_beam[LEFT]->SetRot(chassisRot);
    m_beam[LEFT]->SetMass(GetBalancerBeamMass());
    m_beam[LEFT]->SetInertiaXX(GetBalancerBeamInertia());
    chassis->GetSystem()->AddBody(m_beam[LEFT]);

    // Attach left balancer to chassis through a revolute joint and set joint limits
    m_balancer_joint[LEFT] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_balancer_joint[LEFT]->SetNameString(m_name + "_rev_balancer_L");
    m_balancer_joint[LEFT]->GetLimit_Rz().SetActive(true);
    m_balancer_joint[LEFT]->GetLimit_Rz().SetMin(-GetBalancerMaxPitch());
    m_balancer_joint[LEFT]->GetLimit_Rz().SetMax(+GetBalancerMaxPitch());
    m_balancer_joint[LEFT]->Initialize(m_beam[LEFT], chassis, ChCoordsys<>(m_pointsL[REVOLUTE], joint_rot));
    chassis->GetSystem()->AddLink(m_balancer_joint[LEFT]);

    // Create right side beam body
    m_beam[RIGHT] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_beam[RIGHT]->SetNameString(m_name + "_balancer_R");
    m_beam[RIGHT]->SetPos(m_pointsR[BEAM]);
    m_beam[RIGHT]->SetRot(chassisRot);
    m_beam[RIGHT]->SetMass(GetBalancerBeamMass());
    m_beam[RIGHT]->SetInertiaXX(GetBalancerBeamInertia());
    chassis->GetSystem()->AddBody(m_beam[RIGHT]);

    // Attach right balancer to chassis through a revolute joint and set joint limits
    m_balancer_joint[RIGHT] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_balancer_joint[RIGHT]->SetNameString(m_name + "_rev_balancer_R");
    m_balancer_joint[RIGHT]->GetLimit_Rz().SetActive(true);
    m_balancer_joint[RIGHT]->GetLimit_Rz().SetMin(-GetBalancerMaxPitch());
    m_balancer_joint[RIGHT]->GetLimit_Rz().SetMax(+GetBalancerMaxPitch());
    m_balancer_joint[RIGHT]->Initialize(m_beam[RIGHT], chassis, ChCoordsys<>(m_pointsR[REVOLUTE], joint_rot));
    chassis->GetSystem()->AddLink(m_balancer_joint[RIGHT]);
}

// -----------------------------------------------------------------------------

double ChBalancer::GetMass() const {
    return 2 * GetBalancerBeamMass();
}

ChVector<> ChBalancer::GetCOMPos() const {
    ChVector<> com = GetBalancerBeamMass() * (m_beam[LEFT]->GetPos() + m_beam[RIGHT]->GetPos());
    return com / GetMass();
}

// -----------------------------------------------------------------------------

void ChBalancer::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    auto box_left = chrono_types::make_shared<ChBoxShape>();
    box_left->GetBoxGeometry().SetLengths(GetBalancerBeamDimensions());
    m_beam[LEFT]->AddAsset(box_left);
    m_beam[LEFT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.2f, 0.2f, 0.2f));

    auto box_right = chrono_types::make_shared<ChBoxShape>();
    box_right->GetBoxGeometry().SetLengths(GetBalancerBeamDimensions());
    m_beam[RIGHT]->AddAsset(box_right);
    m_beam[RIGHT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.2f, 0.2f, 0.2f));
}

void ChBalancer::RemoveVisualizationAssets() {
    m_beam[LEFT]->GetAssets().clear();
    m_beam[RIGHT]->GetAssets().clear();
}

}  // end namespace vehicle
}  // end namespace chrono
