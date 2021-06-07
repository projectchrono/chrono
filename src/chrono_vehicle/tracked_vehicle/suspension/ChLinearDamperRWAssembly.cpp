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
// -----------------------------------------------------------------------------
ChLinearDamperRWAssembly::ChLinearDamperRWAssembly(const std::string& name, bool has_shock)
    : ChRoadWheelAssembly(name, has_shock) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                          const ChVector<>& location,
                                          ChTrackAssembly* track) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> susp_to_abs(location);
    susp_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

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
    ChVector<> u = susp_to_abs.GetPos() - points[ARM_CHASSIS];
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

    // Create and initialize the revolute joint between arm and chassis.
    // The axis of rotation is the y axis of the suspension reference frame.
    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis, m_arm,
                           ChCoordsys<>(points[ARM_CHASSIS], susp_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the rotational spring torque element.
    m_spring = chrono_types::make_shared<ChLinkRotSpringCB>();
    m_spring->SetNameString(m_name + "_spring");
    m_spring->Initialize(chassis, m_arm, ChCoordsys<>(points[ARM_CHASSIS], susp_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    m_spring->RegisterTorqueFunctor(GetSpringTorqueFunctor());
    chassis->GetSystem()->AddLink(m_spring);

    // Create and initialize the translational shock force element.
    if (m_has_shock) {
        m_shock = chrono_types::make_shared<ChLinkTSDA>();
        m_shock->SetNameString(m_name + "_shock");
        m_shock->Initialize(chassis, m_arm, false, points[SHOCK_C], points[SHOCK_A]);
        m_shock->RegisterForceFunctor(GetShockForceFunctor());
        chassis->GetSystem()->AddLink(m_shock);
    }

    // Invoke the base class implementation. This initializes the associated road wheel.
    // Note: we must call this here, after the m_arm body is created.
    ChRoadWheelAssembly::Initialize(chassis, location, track);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChLinearDamperRWAssembly::GetMass() const {
    return GetArmMass() + m_road_wheel->GetWheelMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChLinearDamperRWAssembly::GetCarrierAngle() const {
    return m_spring->GetRelAngle();
}

// -----------------------------------------------------------------------------
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
// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::LogConstraintViolations() {
    ChVectorDynamic<> C = m_revolute->GetC();
    GetLog() << "  Arm-chassis revolute\n";
    GetLog() << "  " << C(0) << "  ";
    GetLog() << "  " << C(1) << "  ";
    GetLog() << "  " << C(2) << "  ";
    GetLog() << "  " << C(3) << "  ";
    GetLog() << "  " << C(4) << "\n";

    m_road_wheel->LogConstraintViolations();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChRoadWheelAssembly::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_arm);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkRotSpringCB>> rot_springs;
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
    joints.push_back(m_revolute);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkRotSpringCB>> rot_springs;
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
