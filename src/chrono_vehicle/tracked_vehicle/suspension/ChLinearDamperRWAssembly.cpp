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

    // Sanity check.
    assert(points[ARM_WHEEL].x == susp_to_abs.GetPos().x && points[ARM_WHEEL].z == susp_to_abs.GetPos().z);

    // Create the trailing arm body. The reference frame of the arm body has its
    // x-axis aligned with the line between the arm-chassis connection point and
    // the arm-wheel connection point.
    ChVector<> u = susp_to_abs.GetPos() - points[ARM_CHASSIS];
    u.Normalize();
    ChVector<> w = Vcross(u, susp_to_abs.GetA().Get_A_Yaxis());
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
    AddVisualizationArm(susp_to_abs.GetPos(), points[ARM], points[ARM_WHEEL], points[ARM_CHASSIS], points[SHOCK_A]);
    chassis->GetSystem()->AddBody(m_arm);

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
void ChLinearDamperRWAssembly::AddVisualizationArm(const ChVector<>& pt_O,
                                                   const ChVector<>& pt_A,
                                                   const ChVector<>& pt_AW,
                                                   const ChVector<>& pt_AC,
                                                   const ChVector<>& pt_AS) {
    static const double threshold2 = 1e-6;
    double radius = GetArmVisRadius();

    // Express hardpoint locations in body frame.
    ChVector<> p_O = m_arm->TransformPointParentToLocal(pt_O);
    ChVector<> p_A = m_arm->TransformPointParentToLocal(pt_A);
    ChVector<> p_AC = m_arm->TransformPointParentToLocal(pt_AC);
    ChVector<> p_AW = m_arm->TransformPointParentToLocal(pt_AW);
    ChVector<> p_AS = m_arm->TransformPointParentToLocal(pt_AS);

    if ((p_A - p_AW).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_A;
        cyl->GetCylinderGeometry().p2 = p_AW;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    if ((p_A - p_AC).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_A;
        cyl->GetCylinderGeometry().p2 = p_AC;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    if ((p_A - p_AS).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        cyl->GetCylinderGeometry().p1 = p_A;
        cyl->GetCylinderGeometry().p2 = p_AS;
        cyl->GetCylinderGeometry().rad = 0.75 * radius;
        m_arm->AddAsset(cyl);
    }

    if ((p_O - p_AW).Length2() > threshold2) {
        auto cyl = std::make_shared<ChCylinderShape>();
        double len = (p_O - p_AW).Length();
        cyl->GetCylinderGeometry().p1 = p_O;
        cyl->GetCylinderGeometry().p2 = p_AW + (p_AW - p_O) * radius/len;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.6f, 0.3f));
    m_arm->AddAsset(col);
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
