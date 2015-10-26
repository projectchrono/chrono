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
ChLinearDamperRWAssembly::ChLinearDamperRWAssembly(const std::string& name) : ChRoadWheelAssembly(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::Initialize(ChSharedPtr<ChBodyAuxRef> chassis, const ChVector<>& location) {
    // Invoke the base class implementation
    ChRoadWheelAssembly::Initialize(chassis, location);

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
    ChVector<> u = susp_to_abs.GetPos() - points[ARM_CHASSIS];
    u.Normalize();
    ChVector<> w = Vcross(u, susp_to_abs.GetA().Get_A_Yaxis());
    w.Normalize();
    ChVector<> v = Vcross(w, u);
    ChMatrix33<> rot;
    rot.Set_A_axis(u, v, w);

    m_arm = ChSharedPtr<ChBody>(new ChBody(chassis->GetSystem()->GetContactMethod()));
    m_arm->SetNameString(m_name + "_arm");
    m_arm->SetPos(points[ARM]);
    m_arm->SetRot(rot);
    m_arm->SetMass(GetArmMass());
    m_arm->SetInertiaXX(GetArmInertia());
    AddVisualizationArm(points[ARM], susp_to_abs.GetPos(), points[ARM_CHASSIS]);
    chassis->GetSystem()->AddBody(m_arm);

    // Create and initialize the revolute joint between arm and chassis.
    // The axis of rotation is the y axis of the suspension reference frame.
    m_revolute = ChSharedPtr<ChLinkLockRevolute>(new ChLinkLockRevolute);
    m_revolute->SetNameString(m_name + "_revolute");
    m_revolute->Initialize(chassis, m_arm,
                           ChCoordsys<>(points[ARM_CHASSIS], susp_to_abs.GetRot() * Q_from_AngX(CH_C_PI_2)));
    // Add torsional spring associated with m_revolute
    m_revolute->SetForce_Rz(GetTorsionForceFunction());
    chassis->GetSystem()->AddLink(m_revolute);

    // Create and initialize the tensioner force element.
    m_shock = ChSharedPtr<ChLinkSpringCB>(new ChLinkSpringCB);
    m_shock->SetNameString(m_name + "_shock");
    m_shock->Initialize(chassis, m_arm, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock->Set_SpringCallback(GetShockForceCallback());
    chassis->GetSystem()->AddLink(m_shock);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChLinearDamperRWAssembly::AddVisualizationArm(const ChVector<>& pt_A,
                                                   const ChVector<>& pt_AW,
                                                   const ChVector<>& pt_AC) {
    static const double threshold2 = 1e-6;
    double radius = GetArmVisRadius();

    // Express hardpoint locations in body frame.
    ChVector<> p_A = m_arm->TransformPointParentToLocal(pt_A);
    ChVector<> p_AC = m_arm->TransformPointParentToLocal(pt_AC);
    ChVector<> p_AW = m_arm->TransformPointParentToLocal(pt_AW);

    if ((p_A - p_AW).Length2() > threshold2) {
        ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
        cyl->GetCylinderGeometry().p1 = p_A;
        cyl->GetCylinderGeometry().p2 = p_AW;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    if ((p_A - p_AC).Length2() > threshold2) {
        ChSharedPtr<ChCylinderShape> cyl(new ChCylinderShape);
        cyl->GetCylinderGeometry().p1 = p_A;
        cyl->GetCylinderGeometry().p2 = p_AC;
        cyl->GetCylinderGeometry().rad = radius;
        m_arm->AddAsset(cyl);
    }

    ChSharedPtr<ChColorAsset> col(new ChColorAsset);
    col->SetColor(ChColor(0.2f, 0.6f, 0.3f));
    m_arm->AddAsset(col);
}

}  // end namespace vehicle
}  // end namespace chrono
