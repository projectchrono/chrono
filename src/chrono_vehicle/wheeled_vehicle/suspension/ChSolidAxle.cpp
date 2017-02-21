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
// Authors: Daniel Melanz, Radu Serban
// =============================================================================
//
// Base class for a solid axle suspension modeled with bodies and constraints.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSolidAxle::m_pointNames[] = {"SHOCK_A    ",
                                                 "SHOCK_C    ",
                                                 "KNUCKLE_L  ",
                                                 "KNUCKLE_U  ",
                                                 "LL_A       ",
                                                 "LL_C       ",
                                                 "UL_A       ",
                                                 "UL_C       ",
                                                 "SPRING_A   ",
                                                 "SPRING_C   ",
                                                 "TIEROD_C   ",
                                                 "TIEROD_K   ",
                                                 "SPINDLE    ",
                                                 "KNUCKLE_CM ",
                                                 "LL_CM      ",
                                                 "UL_CM      "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSolidAxle::ChSolidAxle(const std::string& name) : ChSuspension(name) {
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::Initialize(std::shared_ptr<ChBodyAuxRef> chassis,
                             const ChVector<>& location,
                             std::shared_ptr<ChBody> tierod_body,
                             double left_ang_vel,
                             double right_ang_vel) {
    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());

    // Transform the location of the axle body COM to absolute frame.
    ChVector<> axleCOM_local = getAxleTubeCOM();
    ChVector<> axleCOM = suspension_to_abs.TransformLocalToParent(axleCOM_local);

    // Calculate end points on the axle body, expressed in the absolute frame
    // (for visualization)
    ChVector<> midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
    ChVector<> outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    m_axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    // Create and initialize the axle body.
    m_axleTube = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_axleTube->SetNameString(m_name + "_axleTube");
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetSystem()->AddBody(m_axleTube);

    // Calculate end points on the tierod body, expressed in the absolute frame
    // (for visualization)
    ChVector<> tierodOuter_local(getLocation(TIEROD_K));
    m_tierodOuterL = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);
    tierodOuter_local.y() = -tierodOuter_local.y();
    m_tierodOuterR = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);

    // Create and initialize the tierod body.
    m_tierod = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_tierod->SetNameString(m_name + "_tierodBody");
    m_tierod->SetPos((m_tierodOuterL + m_tierodOuterR) / 2);
    m_tierod->SetRot(chassis->GetFrame_REF_to_abs().GetRot());
    m_tierod->SetMass(getTierodMass());
    m_tierod->SetInertiaXX(getTierodInertia());
    chassis->GetSystem()->AddBody(m_tierod);

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

void ChSolidAxle::InitializeSide(VehicleSide side,
                                 std::shared_ptr<ChBodyAuxRef> chassis,
                                 std::shared_ptr<ChBody> tierod_body,
                                 const std::vector<ChVector<> >& points,
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

    // Create and initialize upper link body.
    // Determine the rotation matrix of the upper link based on the plane of the hard points
    // (z-axis along the length of the upper link)
    v = Vcross(points[UL_A] - points[LL_A], points[UL_C] - points[LL_A]);
    v.Normalize();
    w = points[UL_C] - points[UL_A];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_upperLink[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upperLink[side]->SetNameString(m_name + "_upperLink" + suffix);
    m_upperLink[side]->SetPos(points[UL_CM]);
    m_upperLink[side]->SetRot(rot);
    m_upperLink[side]->SetMass(getULMass());
    m_upperLink[side]->SetInertiaXX(getULInertia());
    chassis->GetSystem()->AddBody(m_upperLink[side]);

    // Create and initialize the universal joint between chassis and upper link.
    m_universalUpperLink[side] = std::make_shared<ChLinkUniversal>();
    m_universalUpperLink[side]->SetNameString(m_name + "_universalUpperLink" + suffix);
    m_universalUpperLink[side]->Initialize(chassis, m_upperLink[side], ChFrame<>(points[UL_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalUpperLink[side]);

    // Create and initialize lower link body.
    // Determine the rotation matrix of the lower link based on the plane of the hard points
    // (z-axis along the length of the lower link)
    v = Vcross(points[LL_C] - points[UL_A], points[LL_A] - points[UL_A]);
    v.Normalize();
    w = points[LL_A] - points[LL_C];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_lowerLink[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_lowerLink[side]->SetNameString(m_name + "_lowerLink" + suffix);
    m_lowerLink[side]->SetPos(points[LL_CM]);
    m_lowerLink[side]->SetRot(rot);
    m_lowerLink[side]->SetMass(getLLMass());
    m_lowerLink[side]->SetInertiaXX(getLLInertia());
    chassis->GetSystem()->AddBody(m_lowerLink[side]);

    // Create and initialize the universal joint between chassis and lower link.
    m_universalLowerLink[side] = std::make_shared<ChLinkUniversal>();
    m_universalLowerLink[side]->SetNameString(m_name + "_universalLowerLink" + suffix);
    m_universalLowerLink[side]->Initialize(chassis, m_lowerLink[side], ChFrame<>(points[LL_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalLowerLink[side]);

    // Create and initialize the joint between knuckle and tierod (one side has universal, the other has spherical).
    if (side == LEFT) {
        m_sphericalTierod = std::make_shared<ChLinkLockSpherical>();
        m_sphericalTierod->SetNameString(m_name + "_sphericalTierod" + suffix);
        m_sphericalTierod->Initialize(m_tierod, m_knuckle[side], ChCoordsys<>(points[TIEROD_K], QUNIT));
        chassis->GetSystem()->AddLink(m_sphericalTierod);
    } else {
        m_universalTierod = std::make_shared<ChLinkUniversal>();
        m_universalTierod->SetNameString(m_name + "_universalTierod" + suffix);
        m_universalTierod->Initialize(m_tierod, m_knuckle[side],
                                      ChFrame<>(points[TIEROD_K], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));
        chassis->GetSystem()->AddLink(m_universalTierod);
    }

    // Create and initialize the revolute joint between axle and knuckle.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction.
    w = points[KNUCKLE_L] - points[KNUCKLE_U];
    w.Normalize();
    u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_revoluteKingpin[side] = std::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->Initialize(
        m_axleTube, m_knuckle[side], ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the spherical joint between axle and upper link.
    m_sphericalUpperLink[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalUpperLink[side]->SetNameString(m_name + "_sphericalUpperLink" + suffix);
    m_sphericalUpperLink[side]->Initialize(m_axleTube, m_upperLink[side], ChCoordsys<>(points[UL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUpperLink[side]);

    // Create and initialize the spherical joint between axle and lower link.
    m_sphericalLowerLink[side] = std::make_shared<ChLinkLockSpherical>();
    m_sphericalLowerLink[side]->SetNameString(m_name + "_sphericalLowerLink" + suffix);
    m_sphericalLowerLink[side]->Initialize(m_axleTube, m_lowerLink[side], ChCoordsys<>(points[LL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLowerLink[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = std::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper
    m_shock[side] = std::make_shared<ChLinkSpringCB>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->Set_SpringCallback(getShockForceCallback());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = std::make_shared<ChLinkSpringCB>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_axleTube, false, points[SPRING_C], points[SPRING_A], false,
                               getSpringRestLength());
    m_spring[side]->Set_SpringCallback(getSpringForceCallback());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the tierod distance constraint between chassis and upright.
    if (side == LEFT) {
      // Create and initialize the draglink body (one side only).
      // Determine the rotation matrix of the draglink based on the plane of the hard points
      // (z-axis along the length of the draglink)
      v = Vcross(points[BELLCRANK_DRAGLINK] - points[LL_A], points[DRAGLINK_C] - points[LL_A]);
      v.Normalize();
      w = points[DRAGLINK_C] - points[BELLCRANK_DRAGLINK];
      w.Normalize();
      u = Vcross(v, w);
      rot.Set_A_axis(u, v, w);

      m_draglink = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
      m_draglink->SetNameString(m_name + "_draglink");
      m_draglink->SetPos((points[DRAGLINK_C] + points[BELLCRANK_DRAGLINK]) / 2);
      m_draglink->SetRot(rot.Get_A_quaternion());
      m_draglink->SetMass(getDraglinkMass());
      m_draglink->SetInertiaXX(getDraglinkInertia());
      chassis->GetSystem()->AddBody(m_draglink);

      // Create and initialize the spherical joint between steering mechanism and draglink.
      m_sphericalDraglink = std::make_shared<ChLinkLockSpherical>();
      m_sphericalDraglink->SetNameString(m_name + "_sphericalDraglink" + suffix);
      m_sphericalDraglink->Initialize(m_draglink, tierod_body, ChCoordsys<>(points[DRAGLINK_C], QUNIT));
      chassis->GetSystem()->AddLink(m_sphericalDraglink);

      // Create and initialize bell crank body (one side only).
      m_bellCrank = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
      m_bellCrank->SetNameString(m_name + "_bellCrank");
      m_bellCrank->SetPos((points[BELLCRANK_DRAGLINK] + points[BELLCRANK_AXLE] + points[BELLCRANK_TIEROD]) / 3);
      m_bellCrank->SetRot(rot.Get_A_quaternion());
      m_bellCrank->SetMass(getBellCrankMass());
      m_bellCrank->SetInertiaXX(getBellCrankInertia());
      chassis->GetSystem()->AddBody(m_bellCrank);

      // Create and initialize the universal joint between draglink and bell crank.
      m_universalDraglink = std::make_shared<ChLinkUniversal>();
      m_universalDraglink->SetNameString(m_name + "_universalDraglink" + suffix);
      m_universalDraglink->Initialize(m_draglink, m_bellCrank, ChFrame<>(points[BELLCRANK_DRAGLINK], rot.Get_A_quaternion()));
      chassis->GetSystem()->AddLink(m_universalDraglink);

      // Create and initialize the revolute joint between bellCrank and axle tube.
      // Determine the joint orientation matrix from the hardpoint locations by
      // constructing a rotation matrix with the z axis along the joint direction.
      w = Vcross(points[BELLCRANK_DRAGLINK] - points[BELLCRANK_AXLE], points[BELLCRANK_TIEROD] - points[BELLCRANK_AXLE]);
      w.Normalize();
      v = points[BELLCRANK_TIEROD] - points[BELLCRANK_DRAGLINK];
      v.Normalize();
      u = Vcross(v, w);
      rot.Set_A_axis(u, v, w);

      m_revoluteBellCrank = std::make_shared<ChLinkLockRevolute>();
      m_revoluteBellCrank->SetNameString(m_name + "_revoluteBellCrank" + suffix);
      m_revoluteBellCrank->Initialize(m_bellCrank, m_axleTube, ChCoordsys<>(points[BELLCRANK_AXLE], rot.Get_A_quaternion()));
      chassis->GetSystem()->AddLink(m_revoluteBellCrank);

      // Create and initialize the point-plane joint between bell crank and tierod.
      m_pointPlaneBellCrank = std::make_shared<ChLinkLockPointPlane>();
      m_pointPlaneBellCrank->SetNameString(m_name + "_pointPlaneBellCrank" + suffix);
      m_pointPlaneBellCrank->Initialize(m_bellCrank, m_tierod, ChCoordsys<>(points[BELLCRANK_TIEROD], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));
      chassis->GetSystem()->AddLink(m_pointPlaneBellCrank);
    }

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
double ChSolidAxle::GetMass() const {
    return getAxleTubeMass() + getTierodMass() + getDraglinkMass() + getBellCrankMass() +
           2 * (getSpindleMass() + getULMass() + getLLMass() + getKnuckleMass());
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::LogConstraintViolations(VehicleSide side) {
    // TODO: Update this to reflect new suspension joints
    // Revolute joints
    {
        ChMatrix<>* C = m_revoluteKingpin[side]->GetC();
        GetLog() << "Kingpin revolute      ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }

    {
        ChMatrix<>* C = m_revoluteBellCrank->GetC();
        GetLog() << "Bell Crank revolute      ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "  ";
        GetLog() << "  " << C->GetElement(4, 0) << "\n";
    }

    // Spherical joints 
    {
        ChMatrix<>* C = m_sphericalUpperLink[side]->GetC();
        GetLog() << "UL spherical          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalLowerLink[side]->GetC();
        GetLog() << "LL spherical          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalTierod->GetC();
        GetLog() << "Tierod spherical          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_sphericalDraglink->GetC();
        GetLog() << "Draglink spherical          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "\n";
    }


    // Universal joints
    {
        ChMatrix<>* C = m_universalUpperLink[side]->GetC();
        GetLog() << "UL universal          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }
    {
        ChMatrix<>* C = m_universalLowerLink[side]->GetC();
        GetLog() << "LL universal          ";
        GetLog() << "  " << C->GetElement(0, 0) << "  ";
        GetLog() << "  " << C->GetElement(1, 0) << "  ";
        GetLog() << "  " << C->GetElement(2, 0) << "  ";
        GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }
    {
      ChMatrix<>* C = m_universalTierod->GetC();
      GetLog() << "Tierod universal          ";
      GetLog() << "  " << C->GetElement(0, 0) << "  ";
      GetLog() << "  " << C->GetElement(1, 0) << "  ";
      GetLog() << "  " << C->GetElement(2, 0) << "  ";
      GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }
    {
      ChMatrix<>* C = m_universalDraglink->GetC();
      GetLog() << "Draglink universal          ";
      GetLog() << "  " << C->GetElement(0, 0) << "  ";
      GetLog() << "  " << C->GetElement(1, 0) << "  ";
      GetLog() << "  " << C->GetElement(2, 0) << "  ";
      GetLog() << "  " << C->GetElement(3, 0) << "\n";
    }

    // Point-plane joints
    {
      ChMatrix<>* C = m_pointPlaneBellCrank->GetC();
      GetLog() << "Bell Crank point-plane          ";
      GetLog() << "  " << C->GetElement(0, 0) << "  ";
      GetLog() << "  " << C->GetElement(1, 0) << "  ";
      GetLog() << "  " << C->GetElement(2, 0) << "  ";
      GetLog() << "  " << C->GetElement(3, 0) << "  ";
      GetLog() << "  " << C->GetElement(4, 0) << "  ";
      GetLog() << "  " << C->GetElement(5, 0) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_axleTube, m_pointsL[LL_A], m_pointsL[UL_A], getLLRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_axleTube, m_pointsR[LL_A], m_pointsR[UL_A], getLLRadius(), ChColor(0.7f, 0.7f, 0.7f));

    AddVisualizationLink(m_tierod, m_tierodOuterL, m_tierodOuterR, getTierodRadius(), ChColor(0.7f, 0.7f, 0.7f));

    AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_C], m_pointsL[BELLCRANK_DRAGLINK], getDraglinkRadius(),
                         ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationBellCrank(m_bellCrank, m_pointsL[BELLCRANK_DRAGLINK], m_pointsL[BELLCRANK_AXLE],
                              m_pointsL[BELLCRANK_TIEROD], getBellCrankRadius(), ChColor(0.0f, 0.7f, 0.7f));

    AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius());
    AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius());

    AddVisualizationLink(m_upperLink[LEFT], m_pointsL[UL_A], m_pointsL[UL_C], getULRadius(), ChColor(0.6f, 0.2f, 0.6f));
    AddVisualizationLink(m_upperLink[RIGHT], m_pointsR[UL_A], m_pointsR[UL_C], getULRadius(),
                         ChColor(0.6f, 0.2f, 0.6f));

    AddVisualizationLink(m_lowerLink[LEFT], m_pointsL[LL_A], m_pointsL[LL_C], getLLRadius(), ChColor(0.2f, 0.6f, 0.2f));
    AddVisualizationLink(m_lowerLink[RIGHT], m_pointsR[LL_A], m_pointsR[LL_C], getLLRadius(),
                         ChColor(0.2f, 0.6f, 0.2f));

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddAsset(std::make_shared<ChPointPointSpring>(0.06, 150, 15));

    m_shock[LEFT]->AddAsset(std::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(std::make_shared<ChPointPointSegment>());
}

void ChSolidAxle::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_axleTube->GetAssets().clear();
    m_tierod->GetAssets().clear();
    m_draglink->GetAssets().clear();
    m_bellCrank->GetAssets().clear();

    m_knuckle[LEFT]->GetAssets().clear();
    m_knuckle[RIGHT]->GetAssets().clear();

    m_upperLink[LEFT]->GetAssets().clear();
    m_upperLink[RIGHT]->GetAssets().clear();

    m_lowerLink[LEFT]->GetAssets().clear();
    m_lowerLink[RIGHT]->GetAssets().clear();

    m_spring[LEFT]->GetAssets().clear();
    m_spring[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
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

void ChSolidAxle::AddVisualizationBellCrank(std::shared_ptr<ChBody> body,
  const ChVector<> pt_D,
  const ChVector<> pt_A,
  const ChVector<> pt_T,
  double radius,
  const ChColor& color) {
  // Express hardpoint locations in body frame.
  ChVector<> p_D = body->TransformPointParentToLocal(pt_D);
  ChVector<> p_A = body->TransformPointParentToLocal(pt_A);
  ChVector<> p_T = body->TransformPointParentToLocal(pt_T);

  auto cyl1 = std::make_shared<ChCylinderShape>();
  cyl1->GetCylinderGeometry().p1 = p_D;
  cyl1->GetCylinderGeometry().p2 = p_A;
  cyl1->GetCylinderGeometry().rad = radius;
  body->AddAsset(cyl1);

  auto cyl2 = std::make_shared<ChCylinderShape>();
  cyl2->GetCylinderGeometry().p1 = p_A;
  cyl2->GetCylinderGeometry().p2 = p_T;
  cyl2->GetCylinderGeometry().rad = radius;
  body->AddAsset(cyl2);

  auto col = std::make_shared<ChColorAsset>();
  col->SetColor(color);
  body->AddAsset(col);
}

void ChSolidAxle::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                          const ChVector<> pt_U,
                                          const ChVector<> pt_L,
                                          const ChVector<> pt_T,
                                          double radius) {
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
