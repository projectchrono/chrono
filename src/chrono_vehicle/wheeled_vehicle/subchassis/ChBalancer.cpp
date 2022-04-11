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

#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"

namespace chrono {
namespace vehicle {

ChBalancer::ChBalancer(const std::string& name) : ChSubchassis(name) {}

ChBalancer::~ChBalancer() {
    auto sys = m_beam[0]->GetSystem();
    if (sys) {
        ChChassis::RemoveJoint(m_balancer_joint[0]);
        ChChassis::RemoveJoint(m_balancer_joint[1]);
    }
}

// -----------------------------------------------------------------------------

void ChBalancer::Initialize(std::shared_ptr<ChChassis> chassis, const ChVector<>& location) {
    m_parent = chassis;
    m_rel_loc = location;

    // Express the subchassis reference frame in the absolute coordinate system
    ChFrame<> to_abs(location);
    to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrame_REF_to_abs().GetRot();

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
    m_balancer_joint[LEFT] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::REVOLUTE, m_name + "_rev_balancer_L", m_beam[LEFT], chassis->GetBody(),
        ChCoordsys<>(m_pointsL[REVOLUTE], joint_rot), GetBushingData());
    chassis->AddJoint(m_balancer_joint[LEFT]);

    if (m_balancer_joint[LEFT]->IsKinematic()) {
        auto rev = std::static_pointer_cast<ChLinkLock>(m_balancer_joint[LEFT]->GetAsLink());
        rev->GetLimit_Rz().SetActive(true);
        rev->GetLimit_Rz().SetMin(-GetBalancerMaxPitch());
        rev->GetLimit_Rz().SetMax(+GetBalancerMaxPitch());
    }

    // Create right side beam body
    m_beam[RIGHT] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_beam[RIGHT]->SetNameString(m_name + "_balancer_R");
    m_beam[RIGHT]->SetPos(m_pointsR[BEAM]);
    m_beam[RIGHT]->SetRot(chassisRot);
    m_beam[RIGHT]->SetMass(GetBalancerBeamMass());
    m_beam[RIGHT]->SetInertiaXX(GetBalancerBeamInertia());
    chassis->GetSystem()->AddBody(m_beam[RIGHT]);

    // Attach right balancer to chassis through a revolute joint and set joint limits
    m_balancer_joint[RIGHT] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::REVOLUTE, m_name + "_rev_balancer_R", m_beam[RIGHT], chassis->GetBody(),
        ChCoordsys<>(m_pointsR[REVOLUTE], joint_rot), GetBushingData());
    chassis->AddJoint(m_balancer_joint[RIGHT]);

    if (m_balancer_joint[RIGHT]->IsKinematic()) {
        auto rev = std::static_pointer_cast<ChLinkLock>(m_balancer_joint[RIGHT]->GetAsLink());
        rev->GetLimit_Rz().SetActive(true);
        rev->GetLimit_Rz().SetMin(-GetBalancerMaxPitch());
        rev->GetLimit_Rz().SetMax(+GetBalancerMaxPitch());
    }
}

void ChBalancer::InitializeInertiaProperties() {
    m_mass = 2 * GetBalancerBeamMass();
}

void ChBalancer::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_beam[LEFT]->GetFrame_COG_to_abs(), m_beam[LEFT]->GetMass(), m_beam[LEFT]->GetInertia());
    composite.AddComponent(m_beam[RIGHT]->GetFrame_COG_to_abs(), m_beam[RIGHT]->GetMass(), m_beam[RIGHT]->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
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

// -----------------------------------------------------------------------------

void ChBalancer::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_beam[0]);
    bodies.push_back(m_beam[1]);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_balancer_joint[0]->IsKinematic() ? joints.push_back(m_balancer_joint[0]->GetAsLink())
                                       : bushings.push_back(m_balancer_joint[0]->GetAsBushing());
    m_balancer_joint[1]->IsKinematic() ? joints.push_back(m_balancer_joint[1]->GetAsLink())
                                       : bushings.push_back(m_balancer_joint[1]->GetAsBushing());
    ChPart::ExportJointList(jsonDocument, joints);
    ChPart::ExportBodyLoadList(jsonDocument, bushings);
}

void ChBalancer::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_beam[0]);
    bodies.push_back(m_beam[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_balancer_joint[0]->IsKinematic() ? joints.push_back(m_balancer_joint[0]->GetAsLink())
                                       : bushings.push_back(m_balancer_joint[0]->GetAsBushing());
    m_balancer_joint[1]->IsKinematic() ? joints.push_back(m_balancer_joint[1]->GetAsLink())
                                       : bushings.push_back(m_balancer_joint[1]->GetAsBushing());
    database.WriteJoints(joints);
    database.WriteBodyLoads(bushings);
}

}  // end namespace vehicle
}  // end namespace chrono
