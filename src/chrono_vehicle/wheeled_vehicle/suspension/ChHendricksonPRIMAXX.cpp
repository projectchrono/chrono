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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Base class for a Hendrickson PRIMAXX EX suspension.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChHendricksonPRIMAXX.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChHendricksonPRIMAXX::m_pointNames[] = {"SPINDLE ",
                                                          "KNUCKLE_L",
                                                          "KNUCKLE_U",
                                                          "TIEROD_C",
                                                          "TIEROD_K",
                                                          "TORQUEROD_C",
                                                          "TORQUEROD_AH",
                                                          "LOWERBEAM_C",
                                                          "LOWERBEAM_AH",
                                                          "LOWERBEAM_TB",
                                                          "SHOCKAH_C",
                                                          "SHOCKAH_AH",
                                                          "SHOCKLB_C",
                                                          "SHOCKLB_LB",
                                                          "KNUCKLE_CM",
                                                          "TORQUEROD_CM",
                                                          "LOWERBEAM_CM",
                                                          "TRANSVERSEBEAM_CM"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChHendricksonPRIMAXX::ChHendricksonPRIMAXX(const std::string& name) : ChSuspension(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                                      const ChVector<>& location,
                                      std::shared_ptr<ChBody> tierod_body,
                                      double left_ang_vel,
                                      double right_ang_vel) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform the location of the axle body COM to absolute frame.
    ChVector<> axleCOM_local = getAxlehousingCOM();
    ChVector<> axleCOM = suspension_to_abs.TransformLocalToParent(axleCOM_local);

    // Calculate end points on the axle body, expressed in the absolute frame
    // (for visualization)
    ChVector<> midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
    ChVector<> outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    m_outerL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_outerR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    // Create and initialize the axlehousing body.
    m_axlehousing = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_axlehousing->SetNameString(m_name + "_axlehousing");
    m_axlehousing->SetPos(axleCOM);
    m_axlehousing->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
    m_axlehousing->SetMass(getAxlehousingMass());
    m_axlehousing->SetInertiaXX(getAxlehousingInertia());
    chassis->GetSystem()->AddBody(m_axlehousing);

    // Transform all points and directions to absolute frame
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);

    m_dirsL.resize(NUM_DIRS);
    m_dirsR.resize(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector<> rel_dir = getDirection(static_cast<DirectionId>(i));
        m_dirsL[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
        rel_dir.y() = -rel_dir.y();
        m_dirsR[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
    }

    // Create transverse beam body
    ChVector<> tbCOM_local = getTransversebeamCOM();
    ChVector<> tbCOM = suspension_to_abs.TransformLocalToParent(tbCOM_local);

    m_transversebeam = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_transversebeam->SetNameString(m_name + "_transversebeam");
    m_transversebeam->SetPos(tbCOM);
    m_transversebeam->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
    m_transversebeam->SetMass(getTransversebeamMass());
    m_transversebeam->SetInertiaXX(getTransversebeamInertia());
    chassis->GetSystem()->AddBody(m_transversebeam);

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, m_dirsR, right_ang_vel);
}

void ChHendricksonPRIMAXX::InitializeSide(VehicleSide side,
                                          std::shared_ptr<ChBodyAuxRef> chassis,
                                          std::shared_ptr<ChBody> tierod_body,
                                          const std::vector<ChVector<> >& points,
                                          const std::vector<ChVector<> >& dirs,
                                          double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_knuckle[side]->SetNameString(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(chassisRot);
    m_knuckle[side]->SetMass(getKnuckleMass());
    m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
    chassis->GetSystem()->AddBody(m_knuckle[side]);

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize torque rod body.
    // Determine the rotation matrix of the torque rod based on the plane of the hard points
    // (z-axis along the length of the torque rod)
    v = Vcross(points[TORQUEROD_AH] - points[LOWERBEAM_AH], points[TORQUEROD_C] - points[LOWERBEAM_AH]);
    v.Normalize();
    w = points[TORQUEROD_C] - points[TORQUEROD_AH];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_torquerod[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_torquerod[side]->SetNameString(m_name + "_torquerod" + suffix);
    m_torquerod[side]->SetPos(points[TORQUEROD_CM]);
    m_torquerod[side]->SetRot(rot);
    m_torquerod[side]->SetMass(getTorquerodMass());
    m_torquerod[side]->SetInertiaXX(getTorquerodInertia());
    chassis->GetSystem()->AddBody(m_torquerod[side]);

    // Create and initialize lower beam body.
    // Determine the rotation matrix of the lower link based on the plane of the hard points
    // (z-axis along the length of the lower link)
    v = Vcross(points[LOWERBEAM_C] - points[TORQUEROD_AH], points[LOWERBEAM_AH] - points[TORQUEROD_AH]);
    v.Normalize();
    w = points[LOWERBEAM_C] - points[LOWERBEAM_AH];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_lowerbeam[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_lowerbeam[side]->SetNameString(m_name + "_lowerLink" + suffix);
    m_lowerbeam[side]->SetPos(points[LOWERBEAM_CM]);
    m_lowerbeam[side]->SetRot(rot);
    m_lowerbeam[side]->SetMass(getLowerbeamMass());
    m_lowerbeam[side]->SetInertiaXX(getLowerbeamInertia());
    chassis->GetSystem()->AddBody(m_lowerbeam[side]);

    // Create and initialize the revolute joint between axle and knuckle.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction.
    w = points[KNUCKLE_U] - points[KNUCKLE_L];
    w.Normalize();
    u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_revoluteKingpin[side] = std::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->Initialize(
        m_axlehousing, m_knuckle[side],
        ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the spherical joint between axle housing and torque rod.
    m_sphericalTorquerod[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalTorquerod[side]->SetNameString(m_name + "_sphericalTorquerod" + suffix);
    m_sphericalTorquerod[side]->Initialize(m_axlehousing, m_torquerod[side], ChCoordsys<>(points[TORQUEROD_AH], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTorquerod[side]);

    // Create and initialize the spherical joint between axlehousing and lower beam.
    m_sphericalLowerbeam[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalLowerbeam[side]->SetNameString(m_name + "_sphericalLowerbeam" + suffix);
    m_sphericalLowerbeam[side]->Initialize(m_axlehousing, m_lowerbeam[side], ChCoordsys<>(points[LOWERBEAM_AH], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLowerbeam[side]);

    // Create and initialize the revolute joint between chassis and torque rod.
    ChCoordsys<> revTR_csys(points[TORQUEROD_C], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revoluteTorquerod[side] = std::make_shared<ChLinkLockRevolute>();
    m_revoluteTorquerod[side]->SetNameString(m_name + "_revoluteTorquerod" + suffix);
    m_revoluteTorquerod[side]->Initialize(chassis, m_torquerod[side], revTR_csys);
    chassis->GetSystem()->AddLink(m_revoluteTorquerod[side]);

    // Create and initialize the revolute joint between chassis and lower beam.
    ChCoordsys<> revLB_csys(points[LOWERBEAM_C], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revoluteLowerbeam[side] = std::make_shared<ChLinkLockRevolute>();
    m_revoluteLowerbeam[side]->SetNameString(m_name + "_revoluteLowerbeam" + suffix);
    m_revoluteLowerbeam[side]->Initialize(chassis, m_lowerbeam[side], revLB_csys);
    chassis->GetSystem()->AddLink(m_revoluteLowerbeam[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));

    m_revolute[side] = std::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper between axle housing and chassis
    m_shockAH[side] = std::make_shared<ChLinkSpringCB>();
    m_shockAH[side]->SetNameString(m_name + "_shockAH" + suffix);
    m_shockAH[side]->Initialize(chassis, m_axlehousing, false, points[SHOCKAH_C], points[SHOCKAH_AH]);
    m_shockAH[side]->Set_SpringCallback(getShockAHForceCallback());
    chassis->GetSystem()->AddLink(m_shockAH[side]);

    // Create and initialize the spring/damper between lower beam and chassis
    m_shockLB[side] = std::make_shared<ChLinkSpringCB>();
    m_shockLB[side]->SetNameString(m_name + "_shockLB" + suffix);
    m_shockLB[side]->Initialize(chassis, m_axlehousing, false, points[SHOCKLB_C], points[SHOCKLB_LB]);
    m_shockLB[side]->Set_SpringCallback(getShockLBForceCallback());
    chassis->GetSystem()->AddLink(m_shockLB[side]);

    // Create and initialize the tierod distance constraint between chassis and upright.
    m_distTierod[side] = std::make_shared<ChLinkDistance>();
    m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
    m_distTierod[side]->Initialize(tierod_body, m_knuckle[side], false, points[TIEROD_C], points[TIEROD_K]);
    chassis->GetSystem()->AddLink(m_distTierod[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
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
double ChHendricksonPRIMAXX::GetMass() const {
    return getAxlehousingMass() + getTransversebeamMass() +
           2 * (getSpindleMass() + getKnuckleMass() + getTorquerodMass() + getLowerbeamMass());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChMatrix<>* C = m_revolute[side]->GetC();
        GetLog() << "Spindle revolute      ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_revoluteKingpin[side]->GetC();
        GetLog() << "Kingpin revolute      ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }
    // Spherical joints
    {
        ChMatrix<>* C = m_sphericalTorquerod[side]->GetC();
        GetLog() << "Torquerod spherical          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalLowerbeam[side]->GetC();
        GetLog() << "Lowerbeam spherical          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }

    {
      ChMatrix<>* C = m_revoluteLowerbeam[side]->GetC();
      GetLog() << "Lowerbeam revolute          ";
      GetLog() << "  " << C->GetElement(0, 0) << "  ";
      GetLog() << "  " << C->GetElement(1, 0) << "  ";
      GetLog() << "  " << C->GetElement(2, 0) << "  ";
      GetLog() << "  " << C->GetElement(3, 0) << "  ";
      GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }

    {
      ChMatrix<>* C = m_revoluteTorquerod[side]->GetC();
      GetLog() << "Torquerod revolute          ";
      GetLog() << "  " << C->GetElement(0, 0) << "  ";
      GetLog() << "  " << C->GetElement(1, 0) << "  ";
      GetLog() << "  " << C->GetElement(2, 0) << "  ";
      GetLog() << "  " << C->GetElement(3, 0) << "  ";
      GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }


    // Distance constraint
    GetLog() << "Tierod distance       ";
    GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axlehousing, m_outerL, m_outerR, getAxlehousingRadius(), ChColor(1.0f, 0.0f, 0.0f));
    AddVisualizationLink(m_axlehousing, m_pointsL[LOWERBEAM_AH], m_pointsL[TORQUEROD_AH], getAxlehousingRadius() / 2,
                         ChColor(1.0f, 0.0f, 0.0f));
    AddVisualizationLink(m_axlehousing, m_pointsR[LOWERBEAM_AH], m_pointsR[TORQUEROD_AH], getAxlehousingRadius() / 2,
                         ChColor(1.0f, 0.0f, 0.0f));

    AddVisualizationLink(m_transversebeam, m_pointsL[LOWERBEAM_TB], m_pointsR[LOWERBEAM_TB], getTransversebeamRadius(),
                         ChColor(0.7f, 0.7f, 0.7f));

    AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius());
    AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius());

    AddVisualizationLink(m_torquerod[LEFT], m_pointsL[TORQUEROD_AH], m_pointsL[TORQUEROD_C], getTorquerodRadius(),
                         ChColor(0.6f, 0.2f, 0.6f));
    AddVisualizationLink(m_torquerod[RIGHT], m_pointsR[TORQUEROD_AH], m_pointsR[TORQUEROD_C], getTorquerodRadius(),
                         ChColor(0.6f, 0.2f, 0.6f));

    AddVisualizationLowerBeam(m_lowerbeam[LEFT], m_pointsL[LOWERBEAM_C], m_pointsL[LOWERBEAM_AH],
                              m_pointsL[LOWERBEAM_TB], getLowerbeamRadius(), ChColor(0.2f, 0.6f, 0.2f));
    AddVisualizationLowerBeam(m_lowerbeam[RIGHT], m_pointsR[LOWERBEAM_C], m_pointsR[LOWERBEAM_AH],
                              m_pointsR[LOWERBEAM_TB], getLowerbeamRadius(), ChColor(0.2f, 0.6f, 0.2f));

    // Add visualization for the springs and shocks
    m_shockLB[LEFT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_shockLB[RIGHT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));

    m_shockAH[LEFT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_shockAH[RIGHT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));

    // Add visualization for the tie-rods
    m_distTierod[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distTierod[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_distTierod[LEFT]->AddAsset(std::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
    m_distTierod[RIGHT]->AddAsset(std::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
}

void ChHendricksonPRIMAXX::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_axlehousing->GetAssets().clear();
    m_transversebeam->GetAssets().clear();

    m_knuckle[LEFT]->GetAssets().clear();
    m_knuckle[RIGHT]->GetAssets().clear();

    m_torquerod[LEFT]->GetAssets().clear();
    m_torquerod[RIGHT]->GetAssets().clear();

    m_lowerbeam[LEFT]->GetAssets().clear();
    m_lowerbeam[RIGHT]->GetAssets().clear();

    m_shockLB[LEFT]->GetAssets().clear();
    m_shockLB[RIGHT]->GetAssets().clear();

    m_shockAH[LEFT]->GetAssets().clear();
    m_shockAH[RIGHT]->GetAssets().clear();

    m_distTierod[LEFT]->GetAssets().clear();
    m_distTierod[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
//

void ChHendricksonPRIMAXX::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                const ChVector<> pt_1,
                                                const ChVector<> pt_2,
                                                double radius,
                                                const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

    auto cyl = std::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_1;
    cyl->GetCylinderGeometry().p2 = p_2;
    cyl->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    body->AddAsset(col);
}

void ChHendricksonPRIMAXX::AddVisualizationLowerBeam(std::shared_ptr<ChBody> body,
                                                     const ChVector<> pt_C,
                                                     const ChVector<> pt_AH,
                                                     const ChVector<> pt_TB,
                                                     double radius,
                                                     const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = body->TransformPointParentToLocal(pt_C);
    ChVector<> p_AH = body->TransformPointParentToLocal(pt_AH);
    ChVector<> p_TB = body->TransformPointParentToLocal(pt_TB);

    auto cyl_1 = std::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_C;
    cyl_1->GetCylinderGeometry().p2 = p_AH;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_1);

    auto cyl_2 = std::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_AH;
    cyl_2->GetCylinderGeometry().p2 = p_TB;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddAsset(cyl_2);

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(color);
    body->AddAsset(col);
}

void ChHendricksonPRIMAXX::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                                   const ChVector<> pt_U,
                                                   const ChVector<> pt_L,
                                                   const ChVector<> pt_T,
                                                   double radius)

{
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_U = knuckle->TransformPointParentToLocal(pt_U);
    ChVector<> p_L = knuckle->TransformPointParentToLocal(pt_L);
    ChVector<> p_T = knuckle->TransformPointParentToLocal(pt_T);

    if (p_L.Length2() > threshold2) {
        auto cyl_L = std::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_L;
        cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_L->GetCylinderGeometry().rad = radius;
        knuckle->AddAsset(cyl_L);
    }

    if (p_U.Length2() > threshold2) {
        auto cyl_U = std::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_U->GetCylinderGeometry().rad = radius;
        knuckle->AddAsset(cyl_U);
    }

    if (p_T.Length2() > threshold2) {
        auto cyl_T = std::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_T->GetCylinderGeometry().rad = radius;
        knuckle->AddAsset(cyl_T);
    }

    auto col = std::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    knuckle->AddAsset(col);
}

}  // end namespace vehicle
}  // end namespace chrono
