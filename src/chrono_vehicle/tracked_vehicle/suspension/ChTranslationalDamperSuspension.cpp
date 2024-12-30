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
// Base class for a torsion-bar suspension system using linear dampers (template
// definition).
//
// =============================================================================

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/tracked_vehicle/suspension/ChTranslationalDamperSuspension.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChTranslationalDamperSuspension::ChTranslationalDamperSuspension(const std::string& name, bool has_shock, bool lock_arm)
    : ChTrackSuspension(name, has_shock, lock_arm) {}

ChTranslationalDamperSuspension::~ChTranslationalDamperSuspension() {
    auto sys = m_arm->GetSystem();
    if (sys) {
        sys->Remove(m_arm);
        ChChassis::RemoveJoint(m_joint);
        sys->Remove(m_spring);
        if (m_damper)
            sys->Remove(m_damper);
        if (m_shock)
            sys->Remove(m_shock);
    }
}

// -----------------------------------------------------------------------------
void ChTranslationalDamperSuspension::Initialize(std::shared_ptr<ChChassis> chassis,
                                                 const ChVector3d& location,
                                                 ChTrackAssembly* track) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> susp_to_abs(location);
    susp_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector3d> points(NUM_POINTS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = GetLocation(static_cast<PointId>(i));
        points[i] = susp_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Create the trailing arm body. The reference frame of the arm body has its
    // x-axis aligned with the line between the arm-chassis connection point and
    // the arm-wheel connection point.
    ChVector3d y_dir = susp_to_abs.GetRotMat().GetAxisY();
    ChVector3d u = points[ARM_WHEEL] - points[ARM_CHASSIS];
    u.Normalize();
    ChVector3d w = Vcross(u, y_dir);
    w.Normalize();
    ChVector3d v = Vcross(w, u);
    ChMatrix33<> rot;
    rot.SetFromDirectionAxes(u, v, w);

    m_arm = chrono_types::make_shared<ChBody>();
    m_arm->SetName(m_name + "_arm");
    m_arm->SetPos(points[ARM]);
    m_arm->SetRot(rot);
    m_arm->SetMass(GetArmMass());
    m_arm->SetInertiaXX(GetArmInertia());
    chassis->GetSystem()->AddBody(m_arm);

    // Cache points and directions for arm visualization (expressed in the arm frame)
    m_pO = m_arm->TransformPointParentToLocal(susp_to_abs.GetPos());
    m_pA = m_arm->TransformPointParentToLocal(points[ARM]);
    m_pAW = m_arm->TransformPointParentToLocal(points[ARM_WHEEL]);
    m_pAC = m_arm->TransformPointParentToLocal(points[ARM_CHASSIS]);
    m_pAS = m_arm->TransformPointParentToLocal(points[SHOCK_A]);
    m_dY = m_arm->TransformDirectionParentToLocal(y_dir);

    ChQuaternion<> z2y = susp_to_abs.GetRot() * QuatFromAngleX(-CH_PI_2);

    // Create and initialize the joint between arm and chassis.
    if (m_lock_arm) {
        // Create a weld kinematic joint.
        m_joint =
            chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::LOCK, m_name + "_joint", chassis->GetBody(),
                                                      m_arm, ChFrame<>(points[ARM_CHASSIS], z2y));
    } else {
        // Create a revolute joint or bushing.
        // The axis of rotation is the y axis of the suspension reference frame.
        m_joint = chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_joint",
                                                            chassis->GetBody(), m_arm,
                                                            ChFrame<>(points[ARM_CHASSIS], z2y), getArmBushingData());
    }
    chassis->AddJoint(m_joint);

    // Create and initialize the rotational spring torque element.
    // The reference RSDA frame is aligned with the chassis frame.
    m_spring = chrono_types::make_shared<ChLinkRSDA>();
    m_spring->SetName(m_name + "_spring");
    m_spring->Initialize(chassis->GetBody(), m_arm, ChFrame<>(points[ARM_CHASSIS], z2y));
    m_spring->SetRestAngle(GetSpringRestAngle());
    m_spring->RegisterTorqueFunctor(GetSpringTorqueFunctor());
    chassis->GetSystem()->AddLink(m_spring);

    // Create and initialize the (optional) rotational damper torque element.
    // The reference RSDA frame is aligned with the chassis frame.
    if (GetDamperTorqueFunctor()) {
        m_damper = chrono_types::make_shared<ChLinkRSDA>();
        m_damper->SetName(m_name + "_damper");
        m_damper->Initialize(chassis->GetBody(), m_arm, ChFrame<>(points[ARM_CHASSIS], z2y));
        m_damper->RegisterTorqueFunctor(GetDamperTorqueFunctor());
        chassis->GetSystem()->AddLink(m_damper);
    }

    // Create and initialize the translational shock force element.
    if (m_has_shock) {
        m_shock = chrono_types::make_shared<ChLinkTSDA>();
        m_shock->SetName(m_name + "_shock");
        m_shock->Initialize(chassis->GetBody(), m_arm, false, points[SHOCK_C], points[SHOCK_A]);
        m_shock->RegisterForceFunctor(GetShockForceFunctor());
        chassis->GetSystem()->AddLink(m_shock);
    }

    // Invoke the base class implementation. This initializes the associated road wheel.
    // Note: we must call this here, after the m_arm body is created.
    ChTrackSuspension::Initialize(chassis, location, track);
}

void ChTranslationalDamperSuspension::InitializeInertiaProperties() {
    m_mass = GetArmMass() + m_road_wheel->GetMass();
}

void ChTranslationalDamperSuspension::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_arm->GetFrameCOMToAbs(), m_arm->GetMass(), m_arm->GetInertia());
    composite.AddComponent(m_road_wheel->GetBody()->GetFrameCOMToAbs(), m_road_wheel->GetBody()->GetMass(),
                           m_road_wheel->GetBody()->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

double ChTranslationalDamperSuspension::GetCarrierAngle() const {
    return m_spring->GetAngle();
}

// -----------------------------------------------------------------------------
ChTrackSuspension::ForceTorque ChTranslationalDamperSuspension::ReportSuspensionForce() const {
    ChTrackSuspension::ForceTorque force;

    force.spring_ft = m_spring->GetTorque();
    force.spring_displ = m_spring->GetAngle();
    force.spring_velocity = m_spring->GetVelocity();

    if (m_damper) {
        force.spring_ft += m_damper->GetTorque();
        force.spring_displ += m_damper->GetAngle();
        force.spring_velocity += m_damper->GetVelocity();
    }

    if (m_has_shock) {
        force.shock_ft = m_shock->GetForce();
        force.shock_displ = m_shock->GetLength();
        force.shock_velocity = m_shock->GetVelocity();
    } else {
        force.shock_ft = 0;
        force.shock_displ = 0;
        force.shock_velocity = 0;
    }

    return force;
}

// -----------------------------------------------------------------------------
void ChTranslationalDamperSuspension::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    static const double threshold2 = 1e-6;
    double radius = GetArmVisRadius();

    if ((m_pA - m_pAW).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(m_arm, m_pA, m_pAW, radius);
    }

    if ((m_pA - m_pAC).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(m_arm, m_pA, m_pAC, radius);
    }

    if ((m_pA - m_pAS).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(m_arm, m_pA, m_pAS, 0.75 * radius);
    }

    // Revolute joint (arm-chassis)
    {
        utils::ChBodyGeometry::AddVisualizationCylinder(m_arm,                  //
                                                        m_pAC - radius * m_dY,  //
                                                        m_pAC + radius * m_dY,  //
                                                        1.5 * radius);
    }

    // Revolute joint (arm-wheel)
    if ((m_pO - m_pAW).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChVisualShapeCylinder>();
        double len = (m_pO - m_pAW).Length();
        utils::ChBodyGeometry::AddVisualizationCylinder(m_arm,                                  //
                                                        m_pO,                                   //
                                                        m_pAW + (m_pAW - m_pO) * radius / len,  //
                                                        radius);
    }

    // Visualization of the shock (with default color)
    if (m_has_shock) {
        m_shock->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    }
}

void ChTranslationalDamperSuspension::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_arm);
    if (m_has_shock)
        ChPart::RemoveVisualizationAssets(m_shock);
}

// -----------------------------------------------------------------------------
void ChTranslationalDamperSuspension::LogConstraintViolations() {
    ChVectorDynamic<> C = m_joint->GetConstraintViolation();
    std::cout << "  Arm-chassis joint\n";
    std::cout << "  " << C(0) << "  ";
    std::cout << "  " << C(1) << "  ";
    std::cout << "  " << C(2) << "  ";
    std::cout << "  " << C(3) << "  ";
    std::cout << "  " << C(4) << "\n";

    m_road_wheel->LogConstraintViolations();
}

// -----------------------------------------------------------------------------
void ChTranslationalDamperSuspension::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChTrackSuspension::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_joint->IsKinematic() ? joints.push_back(m_joint->GetAsLink()) : bushings.push_back(m_joint->GetAsBushing());
    ExportJointList(jsonDocument, joints);
    ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkRSDA>> rot_springs;
    rot_springs.push_back(m_spring);
    if (m_damper)
        rot_springs.push_back(m_damper);
    ExportRotSpringList(jsonDocument, rot_springs);

    if (m_has_shock) {
        std::vector<std::shared_ptr<ChLinkTSDA>> lin_springs;
        lin_springs.push_back(m_shock);
        ExportLinSpringList(jsonDocument, lin_springs);
    }
}

void ChTranslationalDamperSuspension::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_joint->IsKinematic() ? joints.push_back(m_joint->GetAsLink()) : bushings.push_back(m_joint->GetAsBushing());
    database.WriteJoints(joints);
    database.WriteBodyLoads(bushings);

    std::vector<std::shared_ptr<ChLinkRSDA>> rot_springs;
    rot_springs.push_back(m_spring);
    if (m_damper)
        rot_springs.push_back(m_damper);
    database.WriteRotSprings(rot_springs);

    if (m_has_shock) {
        std::vector<std::shared_ptr<ChLinkTSDA>> lin_springs;
        lin_springs.push_back(m_shock);
        database.WriteLinSprings(lin_springs);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
