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
//
// Base class for a torsion-bar suspension system using linear dampers (template
// definition).
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/tracked_vehicle/suspension/ChLinearDamperRWAssembly.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChLinearDamperRWAssembly::ChLinearDamperRWAssembly(const std::string& name, bool has_shock)
    : ChRoadWheelAssembly(name), m_has_shock(has_shock) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::Initialize(std::shared_ptr<ChBodyAuxRef> chassis, const ChVector<>& location) {
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
    m_revolute = std::make_shared<ChLinkLockRevolute>();
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis, m_arm,
                           ChCoordsys<>(points[ARM_CHASSIS], susp_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    // Add torsional spring associated with m_revolute
    m_revolute->SetForce_Rz(GetTorsionForceFunction());
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the shock force element.
    if (m_has_shock) {
        m_shock = std::make_shared<ChLinkSpringCB>();
        m_shock->SetNameString(m_name + "_shock");
        m_shock->Initialize(chassis, m_arm, false, points[SHOCK_C], points[SHOCK_A]);
        m_shock->Set_SpringCallback(GetShockForceCallback());
        chassis->GetSystem()->AddLink(m_shock);
    }

    // Invoke the base class implementation. This initializes the associated road wheel.
    // Note: we must call this here, after the m_arm body is created.
    ChRoadWheelAssembly::Initialize(chassis, location);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
double ChLinearDamperRWAssembly::GetMass() const {
    return GetArmMass() + m_road_wheel->GetWheelMass();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    static const double threshold2 = 1e-6;
    double radius = GetArmVisRadius();

    if ((m_pA - m_pAW).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pA;
        cyl->GetCylinderGeometry().p2 = m_pAW;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    if ((m_pA - m_pAC).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pA;
        cyl->GetCylinderGeometry().p2 = m_pAC;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    if ((m_pA - m_pAS).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pA;
        cyl->GetCylinderGeometry().p2 = m_pAS;
        cyl->GetCylinderGeometry().rad = 0.75 * radius;
        m_arm->AddAsset(cyl);
    }

    // Revolute joint (arm-chassis)
    {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = m_pAC - radius * m_dY;
        cyl->GetCylinderGeometry().p2 = m_pAC + radius * m_dY;
        cyl->GetCylinderGeometry().rad = 1.5 * radius;
        m_arm->AddAsset(cyl);
    }

    // Revolute joint (arm-wheel)
    if ((m_pO - m_pAW).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        double len = (m_pO - m_pAW).Length();
        cyl->GetCylinderGeometry().p1 = m_pO;
        cyl->GetCylinderGeometry().p2 = m_pAW + (m_pAW - m_pO) * radius/len;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.6f, 0.3f));
    m_arm->AddAsset(col);

    // Visualization of the shock (with default color)
    if (m_has_shock) {
        m_shock->AddAsset(std::make_shared<ChPointPointSegment>());
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
    ChMatrix<>* C = m_revolute->GetC();
    GetLog() << "  Arm-chassis revolute\n";
    GetLog() << "  " << C->GetElement(0, 0) << "  ";
    GetLog() << "  " << C->GetElement(1, 0) << "  ";
    GetLog() << "  " << C->GetElement(2, 0) << "  ";
    GetLog() << "  " << C->GetElement(3, 0) << "  ";
    GetLog() << "  " << C->GetElement(4, 0) << "\n";

    m_road_wheel->LogConstraintViolations();
}

}  // end namespace vehicle
}  // end namespace chrono
