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
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono_vehicle/ChChassis.h"
#include "chrono_vehicle/wheeled_vehicle/subchassis/ChBalancer.h"

namespace chrono {
namespace vehicle {

ChBalancer::ChBalancer(const std::string& name) : ChSubchassis(name) {}

ChBalancer::~ChBalancer() {
    if (!m_initialized)
        return;

    ChChassis::RemoveJoint(m_balancer_joint[0]);
    ChChassis::RemoveJoint(m_balancer_joint[1]);
}

// -----------------------------------------------------------------------------

void ChBalancer::Construct(std::shared_ptr<ChChassis> chassis, const ChVector3d& location) {
    // Express the subchassis reference frame in the absolute coordinate system
    ChFrame<> to_abs(location);
    to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform all hardpoints to absolute frame
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = GetLocation(static_cast<PointId>(i));
        m_pointsL[i] = to_abs.TransformPointLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Transform pin directions to absolute frame
    ChVector3d rel_dir = GetDirection();
    m_dirL = to_abs.TransformDirectionLocalToParent(rel_dir);
    rel_dir.y() = -rel_dir.y();
    m_dirR = to_abs.TransformDirectionLocalToParent(rel_dir);

    InitializeSide(LEFT, chassis, m_pointsL, m_dirL);
    InitializeSide(RIGHT, chassis, m_pointsR, m_dirR);
}

void ChBalancer::InitializeSide(VehicleSide side,
                                std::shared_ptr<ChChassis> chassis,
                                const std::vector<ChVector3d>& points,
                                const ChVector3d& dir) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrameRefToAbs().GetRot();

    // Orientation of revolute joint
    ChVector3d w = dir.GetNormalized();
    ChVector3d u = VECT_X;
    ChVector3d v = Vcross(w, u);
    v.Normalize();
    u = Vcross(v, w);
    ChMatrix33<> rot(u, v, w);

    ChQuaternion<> joint_rot = chassisRot * rot.GetQuaternion();

    // Create beam body
    m_beam[side] = chrono_types::make_shared<ChBody>();
    m_beam[side]->SetName(m_name + "_balancer" + suffix);
    m_beam[side]->SetTag(m_obj_tag);
    m_beam[side]->SetPos(points[BEAM]);
    m_beam[side]->SetRot(chassisRot);
    m_beam[side]->SetMass(GetBalancerBeamMass());
    m_beam[side]->SetInertiaXX(GetBalancerBeamInertia());
    chassis->GetSystem()->AddBody(m_beam[side]);

    // Attach balancer to chassis through a revolute joint and set joint limits
    m_balancer_joint[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::REVOLUTE, m_name + "_rev_balancer" + suffix, m_beam[side], chassis->GetBody(),
        ChFrame<>(points[REVOLUTE], joint_rot), GetBushingData());
    m_balancer_joint[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_balancer_joint[side]);

    if (m_balancer_joint[side]->IsKinematic()) {
        auto rev = std::static_pointer_cast<ChLinkLock>(m_balancer_joint[side]->GetAsLink());
        rev->LimitRz().SetActive(true);
        rev->LimitRz().SetMin(-GetBalancerMaxPitch());
        rev->LimitRz().SetMax(+GetBalancerMaxPitch());
    }
}

void ChBalancer::InitializeInertiaProperties() {
    m_mass = 2 * GetBalancerBeamMass();
}

void ChBalancer::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_beam[LEFT]->GetFrameCOMToAbs(), m_beam[LEFT]->GetMass(), m_beam[LEFT]->GetInertia());
    composite.AddComponent(m_beam[RIGHT]->GetFrameCOMToAbs(), m_beam[RIGHT]->GetMass(), m_beam[RIGHT]->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------

void ChBalancer::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    const auto& dims = GetBalancerBeamDimensions();

    {
        auto box_left = chrono_types::make_shared<ChVisualShapeBox>(dims);
        m_beam[LEFT]->AddVisualShape(box_left);

        // Orientation of revolute joint
        auto dir = GetDirection();
        ChVector3d w = dir.GetNormalized();
        ChVector3d u = VECT_X;
        ChVector3d v = Vcross(w, u);
        v.Normalize();
        u = Vcross(v, w);
        ChMatrix33<> rot(u, v, w);

        auto p_left = m_beam[LEFT]->TransformPointParentToLocal(m_pointsL[REVOLUTE]);
        auto cyl_left = chrono_types::make_shared<ChVisualShapeCylinder>(dims.z(), 1.4 * dims.y());
        m_beam[LEFT]->AddVisualShape(cyl_left, ChFrame<>(p_left, rot));
    }
    {
        auto box_right = chrono_types::make_shared<ChVisualShapeBox>(dims);
        m_beam[RIGHT]->AddVisualShape(box_right);

        // Orientation of revolute joint
        auto dir = GetDirection();
        dir.y() = -dir.y();
        ChVector3d w = dir.GetNormalized();
        ChVector3d u = VECT_X;
        ChVector3d v = Vcross(w, u);
        v.Normalize();
        u = Vcross(v, w);
        ChMatrix33<> rot(u, v, w);

        auto p_right = m_beam[RIGHT]->TransformPointParentToLocal(m_pointsR[REVOLUTE]);
        auto cyl_right = chrono_types::make_shared<ChVisualShapeCylinder>(dims.z(), 1.4 * dims.y());
        m_beam[RIGHT]->AddVisualShape(cyl_right, ChFrame<>(p_right, rot));
    }
}

void ChBalancer::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_beam[LEFT]);
    ChPart::RemoveVisualizationAssets(m_beam[RIGHT]);
}

// -----------------------------------------------------------------------------

void ChBalancer::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_beam[0]);
    bodies.push_back(m_beam[1]);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_balancer_joint[0]->IsKinematic() ? joints.push_back(m_balancer_joint[0]->GetAsLink())
                                       : bushings.push_back(m_balancer_joint[0]->GetAsBushing());
    m_balancer_joint[1]->IsKinematic() ? joints.push_back(m_balancer_joint[1]->GetAsLink())
                                       : bushings.push_back(m_balancer_joint[1]->GetAsBushing());
    ExportJointList(jsonDocument, joints);
    ExportBodyLoadList(jsonDocument, bushings);
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
