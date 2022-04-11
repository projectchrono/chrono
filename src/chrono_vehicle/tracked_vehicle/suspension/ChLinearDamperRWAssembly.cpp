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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/tracked_vehicle/suspension/ChLinearDamperRWAssembly.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChLinearDamperRWAssembly::ChLinearDamperRWAssembly(const std::string& name, bool has_shock)
    : ChRoadWheelAssembly(name, has_shock) {
}

ChLinearDamperRWAssembly::~ChLinearDamperRWAssembly() {
    auto sys = m_arm->GetSystem();
    if (sys) {
        sys->Remove(m_arm);
        ChChassis::RemoveJoint(m_revolute);
        sys->Remove(m_spring);
        if (m_shock)
            sys->Remove(m_shock);
    }
}

// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::Initialize(std::shared_ptr<ChChassis> chassis,
                                          const ChVector<>& location,
                                          ChTrackAssembly* track) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> susp_to_abs(location);
    susp_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector<> > points(NUM_POINTS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = GetLocation(static_cast<PointId>(i));
        points[i] = susp_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Create the trailing arm body. The reference frame of the arm body has its
    // x-axis aligned with the line between the arm-chassis connection point and
    // the arm-wheel connection point.
    ChVector<> y_dir = susp_to_abs.GetA().Get_A_Yaxis();
    ChVector<> u = points[ARM_WHEEL] - points[ARM_CHASSIS];
    u.Normalize();
    ChVector<> w = Vcross(u, y_dir);
    w.Normalize();
    ChVector<> v = Vcross(w, u);
    ChMatrix33<> rot;
    rot.Set_A_axis(u, v, w);

    m_arm = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_arm->SetNameString(m_name + "_arm");
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

    ChQuaternion<> z2y = susp_to_abs.GetRot() * Q_from_AngX(-CH_C_PI_2);

    // Create and initialize the revolute joint between arm and chassis.
    // The axis of rotation is the y axis of the suspension reference frame.
    m_revolute = chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_revolute",
                                                           chassis->GetBody(), m_arm,
                                                           ChCoordsys<>(points[ARM_CHASSIS], z2y), getArmBushingData());
    chassis->AddJoint(m_revolute);

    // Create and initialize the rotational spring torque element.
    // The reference RSDA frame is aligned with the chassis frame.
    m_spring = chrono_types::make_shared<ChLinkRSDA>();
    m_spring->SetNameString(m_name + "_spring");
    m_spring->Initialize(chassis->GetBody(), m_arm, ChCoordsys<>(points[ARM_CHASSIS], z2y));
    m_spring->RegisterTorqueFunctor(GetSpringTorqueFunctor());
    chassis->GetSystem()->AddLink(m_spring);

    // Create and initialize the translational shock force element.
    if (m_has_shock) {
        m_shock = chrono_types::make_shared<ChLinkTSDA>();
        m_shock->SetNameString(m_name + "_shock");
        m_shock->Initialize(chassis->GetBody(), m_arm, false, points[SHOCK_C], points[SHOCK_A]);
        m_shock->RegisterForceFunctor(GetShockForceFunctor());
        chassis->GetSystem()->AddLink(m_shock);
    }

    // Invoke the base class implementation. This initializes the associated road wheel.
    // Note: we must call this here, after the m_arm body is created.
    ChRoadWheelAssembly::Initialize(chassis, location, track);
}

void ChLinearDamperRWAssembly::InitializeInertiaProperties() {
    m_mass = GetArmMass() + m_road_wheel->GetWheelMass();
}

void ChLinearDamperRWAssembly::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_arm->GetFrame_COG_to_abs(), m_arm->GetMass(), m_arm->GetInertia());
    composite.AddComponent(m_road_wheel->GetWheelBody()->GetFrame_COG_to_abs(), m_road_wheel->GetWheelBody()->GetMass(),
                           m_road_wheel->GetWheelBody()->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

double ChLinearDamperRWAssembly::GetCarrierAngle() const {
    return m_spring->GetAngle();
}

// -----------------------------------------------------------------------------
ChRoadWheelAssembly::ForceTorque ChLinearDamperRWAssembly::ReportSuspensionForce() const {
    ChRoadWheelAssembly::ForceTorque force;

    force.spring_ft = m_spring->GetTorque();
    force.spring_displ = m_spring->GetAngle();
    force.spring_velocity = m_spring->GetVelocity();

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
void ChLinearDamperRWAssembly::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    static const double threshold2 = 1e-6;
    double radius = GetArmVisRadius();

    if ((m_pA - m_pAW).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pA;
        cyl->GetCylinderGeometry().p2 = m_pAW;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    if ((m_pA - m_pAC).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pA;
        cyl->GetCylinderGeometry().p2 = m_pAC;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    if ((m_pA - m_pAS).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pA;
        cyl->GetCylinderGeometry().p2 = m_pAS;
        cyl->GetCylinderGeometry().rad = 0.75 * radius;
        m_arm->AddAsset(cyl);
    }

    // Revolute joint (arm-chassis)
    {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pAC - radius * m_dY;
        cyl->GetCylinderGeometry().p2 = m_pAC + radius * m_dY;
        cyl->GetCylinderGeometry().rad = 1.5 * radius;
        m_arm->AddAsset(cyl);
    }

    // Revolute joint (arm-wheel)
    if ((m_pO - m_pAW).Length2() > threshold2) {
        auto cyl = chrono_types::make_shared<ChCylinderShape>();
        double len = (m_pO - m_pAW).Length();
        cyl->GetCylinderGeometry().p1 = m_pO;
        cyl->GetCylinderGeometry().p2 = m_pAW + (m_pAW - m_pO) * radius/len;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.6f, 0.3f));
    m_arm->AddAsset(col);

    // Visualization of the shock (with default color)
    if (m_has_shock) {
        m_shock->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    }
}

void ChLinearDamperRWAssembly::RemoveVisualizationAssets() {
    m_arm->GetAssets().clear();
    if (m_has_shock)
      m_shock->GetAssets().clear();
}

// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::LogConstraintViolations() {
    ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
    GetLog() << "  Arm-chassis revolute\n";
    GetLog() << "  " << C(0) << "  ";
    GetLog() << "  " << C(1) << "  ";
    GetLog() << "  " << C(2) << "  ";
    GetLog() << "  " << C(3) << "  ";
    GetLog() << "  " << C(4) << "\n";

    m_road_wheel->LogConstraintViolations();
}

// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChRoadWheelAssembly::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_revolute->IsKinematic() ? joints.push_back(m_revolute->GetAsLink())
                              : bushings.push_back(m_revolute->GetAsBushing());
    ChPart::ExportJointList(jsonDocument, joints);
    ChPart::ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkRSDA>> rot_springs;
    rot_springs.push_back(m_spring);
    ChPart::ExportRotSpringList(jsonDocument, rot_springs);

    if (m_has_shock) {
        std::vector<std::shared_ptr<ChLinkTSDA>> lin_springs;
        lin_springs.push_back(m_shock);
        ChPart::ExportLinSpringList(jsonDocument, lin_springs);
    }
}

void ChLinearDamperRWAssembly::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    m_revolute->IsKinematic() ? joints.push_back(m_revolute->GetAsLink())
                              : bushings.push_back(m_revolute->GetAsBushing());
    database.WriteJoints(joints);
    database.WriteBodyLoads(bushings);

    std::vector<std::shared_ptr<ChLinkRSDA>> rot_springs;
    rot_springs.push_back(m_spring);
    database.WriteRotSprings(rot_springs);

    if (m_has_shock) {
        std::vector<std::shared_ptr<ChLinkTSDA>> lin_springs;
        lin_springs.push_back(m_shock);
        database.WriteLinSprings(lin_springs);
    }
}

}  // end namespace vehicle
}  // end namespace chrono
