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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a double-A arm suspension modeled with distance constraints.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// supspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishboneReduced.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleWishboneReduced::ChDoubleWishboneReduced(const std::string& name) : ChSuspension(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                         const ChVector<>& location,
                                         std::shared_ptr<ChBody> tierod_body,
                                         double left_ang_vel,
                                         double right_ang_vel) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform all hardpoints to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    }

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, right_ang_vel);
}

void ChDoubleWishboneReduced::InitializeSide(VehicleSide side,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             std::shared_ptr<ChBody> tierod_body,
                                             const std::vector<ChVector<> >& points,
                                             double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize upright body (same orientation as the chassis)
    m_upright[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upright[side]->SetNameString(m_name + "_upright" + suffix);
    m_upright[side]->SetPos(points[UPRIGHT]);
    m_upright[side]->SetRot(chassisRot);
    m_upright[side]->SetMass(getUprightMass());
    m_upright[side]->SetInertiaXX(getUprightInertia());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Create and initialize joints
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = std::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    m_distUCA_F[side] = std::make_shared<ChLinkDistance>();
    m_distUCA_F[side]->SetNameString(m_name + "_distUCA_F" + suffix);
    m_distUCA_F[side]->Initialize(chassis, m_upright[side], false, points[UCA_F], points[UCA_U]);
    chassis->GetSystem()->AddLink(m_distUCA_F[side]);

    m_distUCA_B[side] = std::make_shared<ChLinkDistance>();
    m_distUCA_B[side]->SetNameString(m_name + "_distUCA_B" + suffix);
    m_distUCA_B[side]->Initialize(chassis, m_upright[side], false, points[UCA_B], points[UCA_U]);
    chassis->GetSystem()->AddLink(m_distUCA_B[side]);

    m_distLCA_F[side] = std::make_shared<ChLinkDistance>();
    m_distLCA_F[side]->SetNameString(m_name + "_distLCA_F" + suffix);
    m_distLCA_F[side]->Initialize(chassis, m_upright[side], false, points[LCA_F], points[LCA_U]);
    chassis->GetSystem()->AddLink(m_distLCA_F[side]);

    m_distLCA_B[side] = std::make_shared<ChLinkDistance>();
    m_distLCA_B[side]->SetNameString(m_name + "_distLCA_B" + suffix);
    m_distLCA_B[side]->Initialize(chassis, m_upright[side], false, points[LCA_B], points[LCA_U]);
    chassis->GetSystem()->AddLink(m_distLCA_B[side]);

    m_distTierod[side] = std::make_shared<ChLinkDistance>();
    m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
    m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
    chassis->GetSystem()->AddLink(m_distTierod[side]);

    // Create and initialize the spring/damper
    m_shock[side] = std::make_shared<ChLinkSpringCB>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_upright[side], false, points[SHOCK_C], points[SHOCK_U]);
    m_shock[side]->Set_SpringRestLength(getSpringRestLength());
    m_shock[side]->Set_SpringCallback(getShockForceCallback());
    chassis->GetSystem()->AddLink(m_shock[side]);

    // Create and initialize the axle shaft and its connection to the spindle.
    // Note that the spindle rotates about the Y axis.
    m_axle[side] = std::make_shared<ChShaft>();
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPos_dt(-ang_vel);
    chassis->GetSystem()->Add(m_axle[side]);

    m_axle_to_spindle[side] = std::make_shared<ChShaftsBody>();
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

// -----------------------------------------------------------------------------
// Get the total mass of the suspension subsystem.
// -----------------------------------------------------------------------------
double ChDoubleWishboneReduced::GetMass() const {
    return 2 * (getSpindleMass() + getUprightMass());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);
    
    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for uprights
    AddVisualizationUpright(m_upright[LEFT], 0.5 * (m_pointsL[SPINDLE] + m_pointsL[UPRIGHT]), m_pointsL[UCA_U],
                            m_pointsL[LCA_U], m_pointsL[TIEROD_U], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], 0.5 * (m_pointsR[SPINDLE] + m_pointsR[UPRIGHT]), m_pointsR[UCA_U],
                            m_pointsR[LCA_U], m_pointsR[TIEROD_U], getUprightRadius());

    // Add visualization for the spring-dampers
    m_shock[LEFT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_shock[RIGHT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));

    // Add visualization for the arm and tie-rod distance constraints
    ChColor col_tierod(0.8f, 0.3f, 0.3f);
    ChColor col_upperarm(0.1f, 0.4f, 0.1f);
    ChColor col_lowerarm(0.1f, 0.1f, 0.4f);

    m_distTierod[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distTierod[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distTierod[LEFT]->AddAsset(std::make_shared<ChColorAsset>(col_tierod));
    m_distTierod[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(col_tierod));

    m_distUCA_F[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distUCA_F[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distUCA_F[LEFT]->AddAsset(std::make_shared<ChColorAsset>(col_upperarm));
    m_distUCA_F[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(col_upperarm));

    m_distUCA_B[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distUCA_B[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distUCA_B[LEFT]->AddAsset(std::make_shared<ChColorAsset>(col_upperarm));
    m_distUCA_B[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(col_upperarm));

    m_distLCA_F[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distLCA_F[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distLCA_F[LEFT]->AddAsset(std::make_shared<ChColorAsset>(col_lowerarm));
    m_distLCA_F[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(col_lowerarm));

    m_distLCA_B[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distLCA_B[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distLCA_B[LEFT]->AddAsset(std::make_shared<ChColorAsset>(col_lowerarm));
    m_distLCA_B[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(col_lowerarm));
}

void ChDoubleWishboneReduced::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_upright[LEFT]->GetAssets().clear();
    m_upright[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();

    m_distTierod[LEFT]->GetAssets().clear();
    m_distTierod[RIGHT]->GetAssets().clear();

    m_distUCA_F[LEFT]->GetAssets().clear();
    m_distUCA_F[RIGHT]->GetAssets().clear();
    m_distUCA_B[LEFT]->GetAssets().clear();
    m_distUCA_B[RIGHT]->GetAssets().clear();

    m_distLCA_F[LEFT]->GetAssets().clear();
    m_distLCA_F[RIGHT]->GetAssets().clear();
    m_distLCA_B[LEFT]->GetAssets().clear();
    m_distLCA_B[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                                      const ChVector<> pt_C,
                                                      const ChVector<> pt_U,
                                                      const ChVector<> pt_L,
                                                      const ChVector<> pt_T,
                                                      double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_C = upright->TransformPointParentToLocal(pt_C);
    ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);
    ChVector<> p_L = upright->TransformPointParentToLocal(pt_L);
    ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);

    if ((p_L - p_C).Length2() > threshold2) {
        auto cyl_L = std::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_L;
        cyl_L->GetCylinderGeometry().p2 = p_C;
        cyl_L->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_L);
    }

    if ((p_U - p_C).Length2() > threshold2) {
        auto cyl_U = std::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = p_C;
        cyl_U->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_U);
    }

    if ((p_T - p_C).Length2() > threshold2) {
        auto cyl_T = std::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = p_C;
        cyl_T->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_T);
    }

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    upright->AddAsset(col);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::LogConstraintViolations(VehicleSide side) {
    // Revolute joint
    {
        ChMatrix<>* C = m_revolute[side]->GetC();
        GetLog() << "Spindle revolute      ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }

    // Distance constraints
    GetLog() << "UCA front distance    ";
    GetLog() << "  " << m_distUCA_F[side]->GetCurrentDistance() - m_distUCA_F[side]->GetImposedDistance() << "\n";

    GetLog() << "UCA back distance     ";
    GetLog() << "  " << m_distUCA_B[side]->GetCurrentDistance() - m_distUCA_B[side]->GetImposedDistance() << "\n";

    GetLog() << "LCA front distance    ";
    GetLog() << "  " << m_distLCA_F[side]->GetCurrentDistance() - m_distLCA_F[side]->GetImposedDistance() << "\n";

    GetLog() << "LCA back distance     ";
    GetLog() << "  " << m_distLCA_B[side]->GetCurrentDistance() - m_distLCA_B[side]->GetImposedDistance() << "\n";

    GetLog() << "Tierod distance       ";
    GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
}

}  // end namespace vehicle
}  // end namespace chrono
