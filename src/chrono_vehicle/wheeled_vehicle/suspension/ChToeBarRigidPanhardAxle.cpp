// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Base class for a steerable solid Panhard axle suspension.
//
// This class is meant for modelling a very simple steerable solid Panhard
// axle. The guiding function is modelled by a ChLinkLockRevolutePrismatic joint
// which allows vertical movement and tilting of the axle tube but no elasticity.
//
// This axle subsystem works with the ChRotaryArm steering subsystem.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
//
// All point locations are assumed to be given for the left half of the
// suspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChToeBarRigidPanhardAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChToeBarRigidPanhardAxle::m_pointNames[] = {
    "SHOCK_A    ", "SHOCK_C    ", "KNUCKLE_L  ", "KNUCKLE_U  ", "KNUCKLE_DRL",
    "SPRING_A   ", "SPRING_C   ", "TIEROD_C   ", "TIEROD_K   ", "SPINDLE    ",
    "KNUCKLE_CM ", "PANHARD_A  ", "PANHARD_C  ", "ANTIROLL_A ", "ANTIROLL_C "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChToeBarRigidPanhardAxle::ChToeBarRigidPanhardAxle(const std::string& name) : ChSuspension(name) {}

ChToeBarRigidPanhardAxle::~ChToeBarRigidPanhardAxle() {
    auto sys = m_axleTubeBody->GetSystem();
    if (sys) {
        sys->Remove(m_axleTubeBody);
        sys->Remove(m_tierodBody);
        sys->Remove(m_draglinkBody);
        sys->Remove(m_axleTubeGuide);
        sys->Remove(m_sphericalTierod);
        sys->Remove(m_sphericalDraglink);
        sys->Remove(m_universalDraglink);
        sys->Remove(m_universalTierod);

        for (int i = 0; i < 2; i++) {
            sys->Remove(m_knuckleBody[i]);
            sys->Remove(m_revoluteKingpin[i]);
            sys->Remove(m_shock[i]);
            sys->Remove(m_spring[i]);
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarRigidPanhardAxle::Initialize(std::shared_ptr<ChChassis> chassis,
                                          std::shared_ptr<ChSubchassis> subchassis,
                                          std::shared_ptr<ChSteering> steering,
                                          const ChVector<>& location,
                                          double left_ang_vel,
                                          double right_ang_vel) {
    ChSuspension::Initialize(chassis, subchassis, steering, location, left_ang_vel, right_ang_vel);

    m_parent = chassis;
    m_rel_loc = location;

    m_left_knuckle_steers = isLeftKnuckleActuated();

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

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

    // Calculate points for visualization of the Panhard rod
    m_panrodOuterA = suspension_to_abs.TransformPointLocalToParent(getLocation(PANHARD_A));
    m_panrodOuterC = suspension_to_abs.TransformPointLocalToParent(getLocation(PANHARD_C));

    ChVector<> arbC_local(getLocation(ANTIROLL_C));
    m_ptARBChassis[LEFT] = suspension_to_abs.TransformPointLocalToParent(arbC_local);
    arbC_local.y() *= -1.0;
    m_ptARBChassis[RIGHT] = suspension_to_abs.TransformPointLocalToParent(arbC_local);

    ChVector<> arbA_local(getLocation(ANTIROLL_A));
    m_ptARBAxle[LEFT] = suspension_to_abs.TransformPointLocalToParent(arbA_local);
    arbA_local.y() *= -1.0;
    m_ptARBAxle[RIGHT] = suspension_to_abs.TransformPointLocalToParent(arbA_local);

    m_ptARBCenter = 0.5 * (m_ptARBChassis[LEFT] + m_ptARBChassis[RIGHT]);

    // Create and initialize the axle body.
    m_axleTubeBody = chrono_types::make_shared<ChBody>();
    m_axleTubeBody->SetNameString(m_name + "_axleTube");
    m_axleTubeBody->SetPos(axleCOM);
    m_axleTubeBody->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_axleTubeBody->SetMass(getAxleTubeMass());
    m_axleTubeBody->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTubeBody);

    // Fix the axle body to the chassis
    m_axleTubeGuide = chrono_types::make_shared<ChLinkLockPlanePlane>();
    m_axleTubeGuide->SetNameString(m_name + "_planePlaneAxleTube");
    const ChQuaternion<>& guideRot = chassis->GetBody()->GetFrame_REF_to_abs().GetRot();
    m_axleTubeGuide->Initialize(chassis->GetBody(), m_axleTubeBody,
                                ChCoordsys<>(axleCOM, guideRot * Q_from_AngY(CH_C_PI_2)));
    chassis->GetBody()->GetSystem()->AddLink(m_axleTubeGuide);

    // Create and initialize the Panhard body.
    ChVector<> ptPanhardCom = 0.5 * (m_panrodOuterA + m_panrodOuterC);
    m_panhardRodBody = chrono_types::make_shared<ChBody>();
    m_panhardRodBody->SetNameString(m_name + "_panhardRod");
    m_panhardRodBody->SetPos(ptPanhardCom);
    m_panhardRodBody->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_panhardRodBody->SetMass(getPanhardRodMass());
    m_panhardRodBody->SetInertiaXX(getPanhardRodInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_panhardRodBody);

    // connect the Panhard rod to the chassis
    m_sphPanhardChassis = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalPanhardChassis", chassis->GetBody(), m_panhardRodBody,
        ChCoordsys<>(m_panrodOuterC, QUNIT));
    chassis->AddJoint(m_sphPanhardChassis);

    // connect the panhard rod to the axle tube
    m_sphPanhardAxle = chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::SPHERICAL,
                                                                 m_name + "_sphericalPanhardAxle", m_axleTubeBody,
                                                                 m_panhardRodBody, ChCoordsys<>(m_panrodOuterA, QUNIT));
    chassis->AddJoint(m_sphPanhardAxle);

    // Calculate end points on the tierod body, expressed in the absolute frame
    // (for visualization)
    ChVector<> tierodOuter_local(getLocation(TIEROD_K));
    m_tierodOuterL = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);
    tierodOuter_local.y() = -tierodOuter_local.y();
    m_tierodOuterR = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);

    // Create and initialize the tierod body.
    m_tierodBody = chrono_types::make_shared<ChBody>();
    m_tierodBody->SetNameString(m_name + "_tierodBody");
    m_tierodBody->SetPos((m_tierodOuterL + m_tierodOuterR) / 2);
    m_tierodBody->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_tierodBody->SetMass(getTierodMass());
    m_tierodBody->SetInertiaXX(getTierodInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_tierodBody);

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
    InitializeSide(LEFT, chassis, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, right_ang_vel);

    // Create connection to steering mechanism
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();

    if (m_left_knuckle_steers) {
        // Create and initialize the draglink body (one side only).
        // Determine the rotation matrix of the draglink based on the plane of the hard points
        // (z-axis along the length of the draglink)
        v = Vcross(m_pointsL[KNUCKLE_DRL], m_pointsL[DRAGLINK_C]);
        v.Normalize();
        w = m_pointsL[DRAGLINK_C] - m_pointsL[KNUCKLE_DRL];
        w.Normalize();
        u = Vcross(v, w);
        rot.Set_A_axis(u, v, w);

        m_draglinkBody = chrono_types::make_shared<ChBody>();
        m_draglinkBody->SetNameString(m_name + "_draglink");
        m_draglinkBody->SetPos((m_pointsL[DRAGLINK_C] + m_pointsL[KNUCKLE_DRL]) / 2);
        m_draglinkBody->SetRot(rot.Get_A_quaternion());
        m_draglinkBody->SetMass(getDraglinkMass());
        m_draglinkBody->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglinkBody);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetNameString(m_name + "_sphericalDraglink" + "_L");
        m_sphericalDraglink->Initialize(m_draglinkBody, tierod_body, ChCoordsys<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize the universal joint between draglink and knuckle
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetNameString(m_name + "_universalDraglink" + "_L");
        m_universalDraglink->Initialize(m_draglinkBody, m_knuckleBody[LEFT],
                                        ChFrame<>(m_pointsL[KNUCKLE_DRL], rot.Get_A_quaternion()));
        chassis->GetBody()->GetSystem()->AddLink(m_universalDraglink);
    } else {
        // Create and initialize the draglink body (one side only).
        // Determine the rotation matrix of the draglink based on the plane of the hard points
        // (z-axis along the length of the draglink)
        v = Vcross(m_pointsR[KNUCKLE_DRL], m_pointsL[DRAGLINK_C]);
        v.Normalize();
        w = m_pointsL[DRAGLINK_C] - m_pointsR[KNUCKLE_DRL];
        w.Normalize();
        u = Vcross(v, w);
        rot.Set_A_axis(u, v, w);

        m_draglinkBody = chrono_types::make_shared<ChBody>();
        m_draglinkBody->SetNameString(m_name + "_draglink");
        m_draglinkBody->SetPos((m_pointsL[DRAGLINK_C] + m_pointsR[KNUCKLE_DRL]) / 2);
        m_draglinkBody->SetRot(rot.Get_A_quaternion());
        m_draglinkBody->SetMass(getDraglinkMass());
        m_draglinkBody->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglinkBody);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetNameString(m_name + "_sphericalDraglink" + "_L");
        m_sphericalDraglink->Initialize(m_draglinkBody, tierod_body, ChCoordsys<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize the universal joint between draglink and knuckle
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetNameString(m_name + "_universalDraglink" + "_R");
        m_universalDraglink->Initialize(m_draglinkBody, m_knuckleBody[RIGHT],
                                        ChFrame<>(m_pointsR[KNUCKLE_DRL], rot.Get_A_quaternion()));
        chassis->GetBody()->GetSystem()->AddLink(m_universalDraglink);
    }
}

void ChToeBarRigidPanhardAxle::InitializeSide(VehicleSide side,
                                              std::shared_ptr<ChChassis> chassis,
                                              const std::vector<ChVector<>>& points,
                                              double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    auto chassisBody = chassis->GetBody();

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassisBody->GetFrame_REF_to_abs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * Q_from_AngZ(sign * getToeAngle()) * Q_from_AngX(sign * getCamberAngle());

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckleBody[side] = chrono_types::make_shared<ChBody>();
    m_knuckleBody[side]->SetNameString(m_name + "_knuckle" + suffix);
    m_knuckleBody[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckleBody[side]->SetRot(spindleRot);
    m_knuckleBody[side]->SetMass(getKnuckleMass());
    m_knuckleBody[side]->SetInertiaXX(getKnuckleInertia());
    chassis->GetSystem()->AddBody(m_knuckleBody[side]);

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = chrono_types::make_shared<ChBody>();
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize the joint between knuckle and tierod (one side has universal, the other has spherical).
    if (side == LEFT) {
        m_sphericalTierod = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalTierod->SetNameString(m_name + "_sphericalTierod" + suffix);
        m_sphericalTierod->Initialize(m_tierodBody, m_knuckleBody[side], ChCoordsys<>(points[TIEROD_K], QUNIT));
        chassis->GetSystem()->AddLink(m_sphericalTierod);
    } else {
        m_universalTierod = chrono_types::make_shared<ChLinkUniversal>();
        m_universalTierod->SetNameString(m_name + "_universalTierod" + suffix);
        m_universalTierod->Initialize(m_tierodBody, m_knuckleBody[side],
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

    m_revoluteKingpin[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->Initialize(
        m_axleTubeBody, m_knuckleBody[side],
        ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], spindleRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckleBody[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis->GetBody(), m_axleTubeBody, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis->GetBody(), m_axleTubeBody, false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPos_dt(-ang_vel);
    chassis->GetSystem()->AddShaft(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftsBody>();
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);

    m_arbBody[side] = chrono_types::make_shared<ChBody>();
    m_arbBody[side]->SetNameString(m_name + "_arb" + suffix);
    m_arbBody[side]->SetPos(0.5 * (points[ANTIROLL_C] + m_ptARBCenter));
    m_arbBody[side]->SetRot(chassisRot);
    m_arbBody[side]->SetMass(getARBMass());
    m_arbBody[side]->SetInertiaXX(getARBInertia());
    chassis->GetSystem()->AddBody(m_arbBody[side]);

    if (side == LEFT) {
        m_revARBChassis = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::REVOLUTE, m_name + "_revARBchassis", chassisBody, m_arbBody[side],
            ChCoordsys<>(m_ptARBCenter, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));
        chassis->AddJoint(m_revARBChassis);
    } else {
        m_revARBLeftRight = chrono_types::make_shared<ChLinkLockRevolute>();
        m_revARBLeftRight->SetNameString(m_name + "_revARBleftRight");
        m_revARBLeftRight->Initialize(m_arbBody[LEFT], m_arbBody[RIGHT],
                                      ChCoordsys<>(m_ptARBCenter, chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));
        chassis->GetSystem()->AddLink(m_revARBLeftRight);

        m_revARBLeftRight->GetForce_Rz().SetActive(1);
        m_revARBLeftRight->GetForce_Rz().SetK(getARBStiffness());
        m_revARBLeftRight->GetForce_Rz().SetR(getARBDamping());
    }

    m_slideARB[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::POINTPLANE, m_name + "_revARBslide" + suffix, m_arbBody[side], m_axleTubeBody,
        ChCoordsys<>(m_ptARBAxle[side], chassisRot * QUNIT));
    chassis->AddJoint(m_slideARB[side]);
}

void ChToeBarRigidPanhardAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + getTierodMass() + getDraglinkMass() + 2 * (getSpindleMass() + getKnuckleMass());
}

void ChToeBarRigidPanhardAxle::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_axleTubeBody->GetFrame_COG_to_abs(), getAxleTubeMass(),
                           ChMatrix33<>(getAxleTubeInertia()));
    composite.AddComponent(m_tierodBody->GetFrame_COG_to_abs(), getTierodMass(), ChMatrix33<>(getTierodInertia()));
    composite.AddComponent(m_draglinkBody->GetFrame_COG_to_abs(), getDraglinkMass(),
                           ChMatrix33<>(getDraglinkInertia()));
    composite.AddComponent(m_knuckleBody[LEFT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckleBody[RIGHT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChToeBarRigidPanhardAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChToeBarRigidPanhardAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarRigidPanhardAxle::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarRigidPanhardAxle::LogConstraintViolations(VehicleSide side) {
    // TODO: Update this to reflect new suspension joints
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteKingpin[side]->GetConstraintViolation();
        GetLog() << "Kingpin revolute      ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }

    {
        ChVectorDynamic<> C = m_sphericalTierod->GetConstraintViolation();
        GetLog() << "Tierod spherical          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalDraglink->GetConstraintViolation();
        GetLog() << "Draglink spherical          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }

    {
        ChVectorDynamic<> C = m_universalTierod->GetConstraintViolation();
        GetLog() << "Tierod universal          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalDraglink->GetConstraintViolation();
        GetLog() << "Draglink universal          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarRigidPanhardAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTubeBody, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_panhardRodBody, m_panrodOuterA, m_panrodOuterC, getPanhardRodRadius(),
                         ChColor(0.5f, 0.7f, 0.3f));

    AddVisualizationLink(m_tierodBody, m_tierodOuterL, m_tierodOuterR, getTierodRadius(), ChColor(0.7f, 0.7f, 0.7f));

    AddVisualizationLink(m_arbBody[LEFT], m_ptARBAxle[LEFT], m_ptARBChassis[LEFT], getARBRadius(),
                         ChColor(0.5f, 7.0f, 0.5f));
    AddVisualizationLink(m_arbBody[LEFT], m_ptARBCenter, m_ptARBChassis[LEFT], getARBRadius(),
                         ChColor(0.5f, 0.7f, 0.5f));

    AddVisualizationLink(m_arbBody[RIGHT], m_ptARBAxle[RIGHT], m_ptARBChassis[RIGHT], getARBRadius(),
                         ChColor(0.7f, 0.5f, 0.5f));
    AddVisualizationLink(m_arbBody[RIGHT], m_ptARBCenter, m_ptARBChassis[RIGHT], getARBRadius(),
                         ChColor(0.7f, 0.5f, 0.5f));

    if (m_left_knuckle_steers) {
        AddVisualizationLink(m_draglinkBody, m_pointsL[DRAGLINK_C], m_pointsL[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    } else {
        AddVisualizationLink(m_draglinkBody, m_pointsL[DRAGLINK_C], m_pointsR[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    }

    AddVisualizationKnuckle(m_knuckleBody[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius());
    AddVisualizationKnuckle(m_knuckleBody[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChToeBarRigidPanhardAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTubeBody);
    ChPart::RemoveVisualizationAssets(m_tierodBody);
    ChPart::RemoveVisualizationAssets(m_draglinkBody);

    ChPart::RemoveVisualizationAssets(m_knuckleBody[LEFT]);
    ChPart::RemoveVisualizationAssets(m_knuckleBody[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarRigidPanhardAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                    const ChVector<> pt_1,
                                                    const ChVector<> pt_2,
                                                    double radius,
                                                    const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

    auto cyl = ChVehicleGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
    cyl->SetColor(color);
}

void ChToeBarRigidPanhardAxle::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
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
        ChVehicleGeometry::AddVisualizationCylinder(knuckle, p_L, VNULL, radius);
    }

    if (p_U.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(knuckle, p_U, VNULL, radius);
    }

    if (p_T.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(knuckle, p_T, VNULL, radius);
    }
}
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarRigidPanhardAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTubeBody);
    bodies.push_back(m_tierodBody);
    bodies.push_back(m_draglinkBody);
    bodies.push_back(m_knuckleBody[0]);
    bodies.push_back(m_knuckleBody[1]);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalTierod);
    joints.push_back(m_sphericalDraglink);
    joints.push_back(m_universalDraglink);
    joints.push_back(m_universalTierod);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChToeBarRigidPanhardAxle::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTubeBody);
    bodies.push_back(m_tierodBody);
    bodies.push_back(m_draglinkBody);
    bodies.push_back(m_knuckleBody[0]);
    bodies.push_back(m_knuckleBody[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalTierod);
    joints.push_back(m_sphericalDraglink);
    joints.push_back(m_universalDraglink);
    joints.push_back(m_universalTierod);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
