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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Base class for a steerable leaf-spring solid axle suspension.
//
// This implementation follows the SAE Spring Design Handbook, which shows the use
// of a kinematic leafspring model made of three links. The three 'links' are
// front leaf / clamp / rear leaf / shackle. The clamp is considered as rigid
// and part of the axle tube in the SAE book. In this chrono model we had to
// split the clamp into two separate bodies clampA and clampB. Both are connected
// via a rotational joint around Z. This was necessary because we need to mimic
// the lateral compliance of the leafspring. The leaves (front and rear) are connected
// to clampA rsp. clampB via rotational joints around Y. The frontleaf also connects
// to the chassis with a spherical joint, while the rearleaf connects to the shackle
// body with a spherical joint. The shackle body connects to the chassis via a
// rotational joint around Y.
//
// The vertical stiffnesses are simulated by the rotational springs of the frontleaf
// and the rearleaf while the lateral stiffnesses are simulated by the rotational
// joints of the clamp bodies.
//
// For the stiffness parameters the user can take the desired translatoric vertical
// stiffness dividedby two (we have two leaves). The preload can be set as a
// vertical force dividedby two. The conversion to the rotary setup is made
// automatically.
//
// The SAE model allows to consider the correct axle movement due to wheel travel.
// It is possible to consider the tie-up mode due to longitudinal forces (breaking).
// It also possible to use different stiffnesses for the front leaf and the rear
// leaf, many practical designs use this to avoid tie-up problems.
//
// This axle subsystem works with the ChRotaryArm steering subsystem.
//
// The suspension subsystem is modeled with respect to a right-handed frame,
// with X pointing towards the front, Y to the left, and Z up (ISO standard).
// The suspension reference frame is assumed to be always aligned with that of
// the vehicle.  When attached to a chassis, only an offset is provided.
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

#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSAEToeBarLeafspringAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSAEToeBarLeafspringAxle::m_pointNames[] = {
    "SHOCK_A     ", "SHOCK_C     ", "KNUCKLE_L   ", "KNUCKLE_U   ", "KNUCKLE_DRL ", "SPRING_A    ",
    "SPRING_C    ", "TIEROD_C    ", "TIEROD_K    ", "SPINDLE     ", "KNUCKLE_CM  ", "CLAMP_A     ",
    "CLAMP_B     ", "FRONT_HANGER", "REAR_HANGER ", "SHACKLE     "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSAEToeBarLeafspringAxle::ChSAEToeBarLeafspringAxle(const std::string& name) : ChSuspension(name) {}

ChSAEToeBarLeafspringAxle::~ChSAEToeBarLeafspringAxle() {
    if (!IsInitialized())
        return;

    auto sys = m_axleTube->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_axleTube);
    sys->Remove(m_tierod);
    sys->Remove(m_draglink);

    sys->Remove(m_sphericalTierod);
    sys->Remove(m_sphericalDraglink);
    sys->Remove(m_universalDraglink);
    sys->Remove(m_universalTierod);

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_knuckle[i]);
        sys->Remove(m_revoluteKingpin[i]);

        sys->Remove(m_shock[i]);
        sys->Remove(m_spring[i]);

        sys->Remove(m_shackle[i]);
        sys->Remove(m_frontleaf[i]);
        sys->Remove(m_rearleaf[i]);
        sys->Remove(m_clampA[i]);
        sys->Remove(m_clampB[i]);

        sys->Remove(m_frontleafSph[i]);
        sys->Remove(m_rearleafSph[i]);

        ChChassis::RemoveJoint(m_shackleRev[i]);
        ChChassis::RemoveJoint(m_frontleafRev[i]);
        ChChassis::RemoveJoint(m_rearleafRev[i]);
        ChChassis::RemoveJoint(m_clampARev[i]);
        ChChassis::RemoveJoint(m_clampBRev[i]);

        sys->Remove(m_latRotSpringA[i]);
        sys->Remove(m_latRotSpringB[i]);
        sys->Remove(m_vertRotSpringA[i]);
        sys->Remove(m_vertRotSpringB[i]);
    }
}

// -----------------------------------------------------------------------------
void ChSAEToeBarLeafspringAxle::Construct(std::shared_ptr<ChChassis> chassis,
                                          std::shared_ptr<ChSubchassis> subchassis,
                                          std::shared_ptr<ChSteering> steering,
                                          const ChVector3d& location,
                                          double left_ang_vel,
                                          double right_ang_vel) {
    m_left_knuckle_steers = isLeftKnuckleActuated();

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform the location of the axle body COM to absolute frame.
    ChVector3d axleCOM_local = getAxleTubeCOM();
    ChVector3d axleCOM = suspension_to_abs.TransformPointLocalToParent(axleCOM_local);

    // Calculate end points on the axle body, expressed in the absolute frame
    // (for visualization)
    ChVector3d midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
    ChVector3d outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    m_axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    // Create and initialize the axle body.
    m_axleTube = chrono_types::make_shared<ChBody>();
    m_axleTube->SetName(m_name + "_axleTube");
    m_axleTube->SetTag(m_obj_tag);
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTube);

    // Calculate end points on the tierod body, expressed in the absolute frame
    // (for visualization)
    ChVector3d tierodOuter_local(getLocation(TIEROD_K));
    m_tierodOuterL = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);
    tierodOuter_local.y() = -tierodOuter_local.y();
    m_tierodOuterR = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);

    // Create and initialize the tierod body.
    m_tierod = chrono_types::make_shared<ChBody>();
    m_tierod->SetName(m_name + "_tierodBody");
    m_tierod->SetTag(m_obj_tag);
    m_tierod->SetPos((m_tierodOuterL + m_tierodOuterR) / 2);
    m_tierod->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_tierod->SetMass(getTierodMass());
    m_tierod->SetInertiaXX(getTierodInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_tierod);

    // Transform all hardpoints to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
    }

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, right_ang_vel);

    // Initialize connections to steering mechanism
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
        rot.SetFromDirectionAxes(u, v, w);

        m_draglink = chrono_types::make_shared<ChBody>();
        m_draglink->SetName(m_name + "_draglink");
        m_draglink->SetTag(m_obj_tag);
        m_draglink->SetPos((m_pointsL[DRAGLINK_C] + m_pointsL[KNUCKLE_DRL]) / 2);
        m_draglink->SetRot(rot.GetQuaternion());
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetName(m_name + "_sphericalDraglink" + "_L");
        m_sphericalDraglink->SetTag(m_obj_tag);
        m_sphericalDraglink->Initialize(m_draglink, tierod_body, ChFrame<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize the universal joint between draglink and knuckle
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetName(m_name + "_universalDraglink" + "_L");
        m_universalDraglink->SetTag(m_obj_tag);
        m_universalDraglink->Initialize(m_draglink, m_knuckle[LEFT],
                                        ChFrame<>(m_pointsL[KNUCKLE_DRL], rot.GetQuaternion()));
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
        rot.SetFromDirectionAxes(u, v, w);

        m_draglink = chrono_types::make_shared<ChBody>();
        m_draglink->SetName(m_name + "_draglink");
        m_draglink->SetTag(m_obj_tag);
        m_draglink->SetPos((m_pointsL[DRAGLINK_C] + m_pointsR[KNUCKLE_DRL]) / 2);
        m_draglink->SetRot(rot.GetQuaternion());
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetName(m_name + "_sphericalDraglink" + "_L");
        m_sphericalDraglink->SetTag(m_obj_tag);
        m_sphericalDraglink->Initialize(m_draglink, tierod_body, ChFrame<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize the universal joint between draglink and knuckle
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetName(m_name + "_universalDraglink" + "_R");
        m_universalDraglink->SetTag(m_obj_tag);
        m_universalDraglink->Initialize(m_draglink, m_knuckle[RIGHT],
                                        ChFrame<>(m_pointsR[KNUCKLE_DRL], rot.GetQuaternion()));
        chassis->GetBody()->GetSystem()->AddLink(m_universalDraglink);
    }
}

void ChSAEToeBarLeafspringAxle::InitializeSide(VehicleSide side,
                                               std::shared_ptr<ChChassis> chassis,
                                               const std::vector<ChVector3d>& points,
                                               double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrameRefToAbs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * QuatFromAngleZ(sign * getToeAngle()) * QuatFromAngleX(sign * getCamberAngle());

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = chrono_types::make_shared<ChBody>();
    m_knuckle[side]->SetName(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetTag(m_obj_tag);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(chassisRot);
    m_knuckle[side]->SetMass(getKnuckleMass());
    m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
    chassis->GetSystem()->AddBody(m_knuckle[side]);

    // Initialize spindle body (same orientation as the chassis)
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());

    // Create and initialize the joint between knuckle and tierod (one side has universal, the other has spherical).
    if (side == LEFT) {
        m_sphericalTierod = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalTierod->SetName(m_name + "_sphericalTierod" + suffix);
        m_sphericalTierod->SetTag(m_obj_tag);
        m_sphericalTierod->Initialize(m_tierod, m_knuckle[side], ChFrame<>(points[TIEROD_K], QUNIT));
        chassis->GetSystem()->AddLink(m_sphericalTierod);
    } else {
        m_universalTierod = chrono_types::make_shared<ChLinkUniversal>();
        m_universalTierod->SetName(m_name + "_universalTierod" + suffix);
        m_universalTierod->SetTag(m_obj_tag);
        m_universalTierod->Initialize(m_tierod, m_knuckle[side],
                                      ChFrame<>(points[TIEROD_K], chassisRot * QuatFromAngleX(CH_PI_2)));
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
    rot.SetFromDirectionAxes(u, v, w);

    m_revoluteKingpin[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetName(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->SetTag(m_obj_tag);
    m_revoluteKingpin[side]->Initialize(m_axleTube, m_knuckle[side],
                                        ChFrame<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.GetQuaternion()));
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the revolute joint between upright and spindle.
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->SetTag(m_obj_tag);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side],
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->SetTag(m_obj_tag);
    m_shock[side]->Initialize(chassis->GetBody(), m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetName(m_name + "_spring" + suffix);
    m_spring[side]->SetTag(m_obj_tag);
    m_spring[side]->Initialize(chassis->GetBody(), m_axleTube, false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetName(m_name + "_axle" + suffix);
    m_axle[side]->SetTag(m_obj_tag);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPosDt(-ang_vel);
    chassis->GetSystem()->AddShaft(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftBodyRotation>();
    m_axle_to_spindle[side]->SetName(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector3d(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);

    // Leafspring related elements
    // Create and initialize the shackle body.
    m_shackle[side] = chrono_types::make_shared<ChBody>();
    m_shackle[side]->SetName(m_name + "_shackle" + suffix);
    m_shackle[side]->SetTag(m_obj_tag);
    m_shackle[side]->SetPos((points[REAR_HANGER] + points[SHACKLE]) / 2.0);
    m_shackle[side]->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_shackle[side]->SetMass(getShackleMass());
    m_shackle[side]->SetInertiaXX(getShackleInertia());
    chassis->GetSystem()->AddBody(m_shackle[side]);

    // chassis-shackle rev joint
    m_shackleRev[side] = chrono_types::make_shared<ChJoint>(
        ChJoint::Type::REVOLUTE, m_name + "_shackleRev" + suffix, m_shackle[side], chassis->GetBody(),
        ChFrame<>(points[REAR_HANGER], chassisRot * QuatFromAngleX(CH_PI_2)), getShackleBushingData());
    m_shackleRev[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_shackleRev[side]);

    // Create and initialize the frontleaf body.
    m_frontleaf[side] = chrono_types::make_shared<ChBody>();
    m_frontleaf[side]->SetName(m_name + "frontleaf" + suffix);
    m_frontleaf[side]->SetTag(m_obj_tag);
    m_frontleaf[side]->SetPos((points[FRONT_HANGER] + points[CLAMP_A]) / 2.0);
    m_frontleaf[side]->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_frontleaf[side]->SetMass(getFrontLeafMass());
    m_frontleaf[side]->SetInertiaXX(getFrontLeafInertia());
    chassis->GetSystem()->AddBody(m_frontleaf[side]);

    // Create and initialize the spherical joint between frontleaf and chassis
    m_frontleafSph[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_frontleafSph[side]->SetName(m_name + "_frontleafSpherical" + suffix);
    m_frontleafSph[side]->SetTag(m_obj_tag);
    m_frontleafSph[side]->Initialize(m_frontleaf[side], chassis->GetBody(), ChFrame<>(points[FRONT_HANGER], QUNIT));
    chassis->GetSystem()->AddLink(m_frontleafSph[side]);

    // Create and initialize the rearleaf body.
    m_rearleaf[side] = chrono_types::make_shared<ChBody>();
    m_rearleaf[side]->SetName(m_name + "rearleaf" + suffix);
    m_rearleaf[side]->SetTag(m_obj_tag);
    m_rearleaf[side]->SetPos((points[SHACKLE] + points[CLAMP_B]) / 2.0);
    m_rearleaf[side]->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_rearleaf[side]->SetMass(getRearLeafMass());
    m_rearleaf[side]->SetInertiaXX(getRearLeafInertia());
    chassis->GetSystem()->AddBody(m_rearleaf[side]);

    // Create and initialize the spherical joint between rearleaf and shackle
    m_rearleafSph[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_rearleafSph[side]->SetName(m_name + "_rearleafSpherical" + suffix);
    m_rearleafSph[side]->SetTag(m_obj_tag);
    m_rearleafSph[side]->Initialize(m_rearleaf[side], m_shackle[side], ChFrame<>(points[SHACKLE], QUNIT));
    chassis->GetSystem()->AddLink(m_rearleafSph[side]);

    // Create and initialize the clampA body.
    ChVector3d mid = (points[CLAMP_A] + points[CLAMP_B]) / 2.0;
    m_clampA[side] = chrono_types::make_shared<ChBody>();
    m_clampA[side]->SetName(m_name + "_clampA" + suffix);
    m_clampA[side]->SetTag(m_obj_tag);
    m_clampA[side]->SetPos((points[CLAMP_A] + mid) / 2.0);
    m_clampA[side]->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_clampA[side]->SetMass(getClampMass());
    m_clampA[side]->SetInertiaXX(getClampInertia());
    chassis->GetSystem()->AddBody(m_clampA[side]);

    // clampA-axleTube rev joint (Z)
    ChFrame<> rev_frame_clampA(points[CLAMP_A], chassisRot);
    m_clampARev[side] =
        chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_clampARev" + suffix, m_clampA[side],
                                           m_axleTube, rev_frame_clampA, getClampBushingData());
    m_clampARev[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_clampARev[side]);

    m_latRotSpringA[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_latRotSpringA[side]->SetTag(m_obj_tag);
    m_latRotSpringA[side]->Initialize(m_clampA[side], m_axleTube, rev_frame_clampA);
    m_latRotSpringA[side]->RegisterTorqueFunctor(getLatTorqueFunctorA());
    chassis->GetSystem()->AddLink(m_latRotSpringA[side]);

    // Create and initialize the clampB body.
    // mid = (points[CLAMP_A] + points[CLAMP_B]) / 2.0;
    m_clampB[side] = chrono_types::make_shared<ChBody>();
    m_clampB[side]->SetName(m_name + "_clampB" + suffix);
    m_clampB[side]->SetTag(m_obj_tag);
    m_clampB[side]->SetPos((points[CLAMP_B] + mid) / 2.0);
    m_clampB[side]->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_clampB[side]->SetMass(getClampMass());
    m_clampB[side]->SetInertiaXX(getClampInertia());
    chassis->GetSystem()->AddBody(m_clampB[side]);

    // clampB-axleTube rev joint (Z)
    ChFrame<> rev_frame_clampB(points[CLAMP_B], chassisRot);
    m_clampBRev[side] =
        chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_clampBRev" + suffix, m_clampB[side],
                                           m_axleTube, rev_frame_clampB, getClampBushingData());
    m_clampBRev[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_clampBRev[side]);

    m_latRotSpringB[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_latRotSpringB[side]->Initialize(m_clampB[side], m_axleTube, rev_frame_clampB);
    m_latRotSpringB[side]->RegisterTorqueFunctor(getLatTorqueFunctorB());
    chassis->GetSystem()->AddLink(m_latRotSpringB[side]);

    // clampB-rearleaf rev joint (Y)
    ChFrame<> rev_frame_rearleaf(points[CLAMP_B], chassisRot * QuatFromAngleX(CH_PI_2));
    m_rearleafRev[side] =
        chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_rearleafRev" + suffix, m_clampB[side],
                                           m_rearleaf[side], rev_frame_rearleaf, getLeafspringBushingData());
    m_rearleafRev[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_rearleafRev[side]);

    m_vertRotSpringB[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_vertRotSpringB[side]->SetTag(m_obj_tag);
    m_vertRotSpringB[side]->Initialize(m_clampB[side], m_rearleaf[side], rev_frame_rearleaf);
    m_vertRotSpringB[side]->RegisterTorqueFunctor(getVertTorqueFunctorB());
    chassis->GetSystem()->AddLink(m_vertRotSpringB[side]);

    // clampA-frontleaf rev joint (Y)
    ChFrame<> rev_frame_frontleaf(points[CLAMP_A], chassisRot * QuatFromAngleX(CH_PI_2));
    m_frontleafRev[side] =
        chrono_types::make_shared<ChJoint>(ChJoint::Type::REVOLUTE, m_name + "_frontleafRev" + suffix, m_clampA[side],
                                           m_frontleaf[side], rev_frame_frontleaf, getLeafspringBushingData());
    m_frontleafRev[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_frontleafRev[side]);

    m_vertRotSpringA[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_vertRotSpringA[side]->SetTag(m_obj_tag);
    m_vertRotSpringA[side]->Initialize(m_clampA[side], m_frontleaf[side], rev_frame_frontleaf);
    m_vertRotSpringA[side]->RegisterTorqueFunctor(getVertTorqueFunctorA());
    chassis->GetSystem()->AddLink(m_vertRotSpringA[side]);
}

void ChSAEToeBarLeafspringAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + getTierodMass() + getDraglinkMass() +
             2 * (getSpindleMass() + getKnuckleMass() + getFrontLeafMass() + getRearLeafMass() + 2 * getClampMass() +
                  getShackleMass());
}

void ChSAEToeBarLeafspringAxle::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());
    ChMatrix33<> inertiaFrontLeaf(getFrontLeafInertia());
    ChMatrix33<> inertiaRearLeaf(getRearLeafInertia());
    ChMatrix33<> inertiaClamp(getClampInertia());
    ChMatrix33<> inertiaShackle(getShackleInertia());

    CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);

    composite.AddComponent(m_knuckle[LEFT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckle[RIGHT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);

    composite.AddComponent(m_frontleaf[LEFT]->GetFrameCOMToAbs(), getFrontLeafMass(), inertiaFrontLeaf);
    composite.AddComponent(m_frontleaf[RIGHT]->GetFrameCOMToAbs(), getFrontLeafMass(), inertiaFrontLeaf);

    composite.AddComponent(m_rearleaf[LEFT]->GetFrameCOMToAbs(), getRearLeafMass(), inertiaRearLeaf);
    composite.AddComponent(m_rearleaf[RIGHT]->GetFrameCOMToAbs(), getRearLeafMass(), inertiaRearLeaf);

    composite.AddComponent(m_clampA[LEFT]->GetFrameCOMToAbs(), getClampMass(), inertiaClamp);
    composite.AddComponent(m_clampA[RIGHT]->GetFrameCOMToAbs(), getClampMass(), inertiaClamp);

    composite.AddComponent(m_clampB[LEFT]->GetFrameCOMToAbs(), getClampMass(), inertiaClamp);
    composite.AddComponent(m_clampB[RIGHT]->GetFrameCOMToAbs(), getClampMass(), inertiaClamp);

    composite.AddComponent(m_shackle[LEFT]->GetFrameCOMToAbs(), getShackleMass(), inertiaShackle);
    composite.AddComponent(m_shackle[RIGHT]->GetFrameCOMToAbs(), getShackleMass(), inertiaShackle);

    composite.AddComponent(m_axleTube->GetFrameCOMToAbs(), getAxleTubeMass(), ChMatrix33<>(getAxleTubeInertia()));
    composite.AddComponent(m_tierod->GetFrameCOMToAbs(), getTierodMass(), ChMatrix33<>(getTierodInertia()));
    composite.AddComponent(m_draglink->GetFrameCOMToAbs(), getDraglinkMass(), ChMatrix33<>(getDraglinkInertia()));

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChSAEToeBarLeafspringAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChSAEToeBarLeafspringAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAEToeBarLeafspringAxle::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAEToeBarLeafspringAxle::LogConstraintViolations(VehicleSide side) {
    // TODO: Update this to reflect new suspension joints
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteKingpin[side]->GetConstraintViolation();
        std::cout << "Kingpin revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    {
        ChVectorDynamic<> C = m_sphericalTierod->GetConstraintViolation();
        std::cout << "Tierod spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalDraglink->GetConstraintViolation();
        std::cout << "Draglink spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }

    {
        ChVectorDynamic<> C = m_universalTierod->GetConstraintViolation();
        std::cout << "Tierod universal          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalDraglink->GetConstraintViolation();
        std::cout << "Draglink universal          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAEToeBarLeafspringAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));

    AddVisualizationLink(m_tierod, m_tierodOuterL, m_tierodOuterR, getTierodRadius(), ChColor(0.7f, 0.7f, 0.7f));

    if (m_left_knuckle_steers) {
        AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_C], m_pointsL[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    } else {
        AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_C], m_pointsR[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    }

    AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius());
    AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.03, 150, 10));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.03, 150, 10));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    double rs = getAxleTubeRadius() / 10.0;

    // frontleaf visualisation
    AddVisualizationLink(m_frontleaf[LEFT], m_pointsL[FRONT_HANGER], m_pointsL[CLAMP_A], rs, ChColor(0.5f, 0.5f, 0.7f));
    AddVisualizationLink(m_frontleaf[RIGHT], m_pointsR[FRONT_HANGER], m_pointsR[CLAMP_A], rs,
                         ChColor(0.5f, 0.5f, 0.7f));

    // clampA visualisation
    AddVisualizationLink(m_clampA[LEFT], m_pointsL[CLAMP_A], (m_pointsL[CLAMP_A] + m_pointsL[CLAMP_B]) / 2.0, rs,
                         ChColor(0.5f, 0.7f, 0.5f));
    AddVisualizationLink(m_clampA[RIGHT], m_pointsR[CLAMP_A], (m_pointsR[CLAMP_A] + m_pointsR[CLAMP_B]) / 2.0, rs,
                         ChColor(0.5f, 0.7f, 0.5f));

    // clampB visualisation
    AddVisualizationLink(m_clampB[LEFT], m_pointsL[CLAMP_B], (m_pointsL[CLAMP_A] + m_pointsL[CLAMP_B]) / 2.0, rs,
                         ChColor(0.5f, 0.5f, 0.7f));
    AddVisualizationLink(m_clampB[RIGHT], m_pointsR[CLAMP_B], (m_pointsR[CLAMP_A] + m_pointsR[CLAMP_B]) / 2.0, rs,
                         ChColor(0.5f, 0.5f, 0.7f));

    // rearleaf visualisation
    AddVisualizationLink(m_rearleaf[LEFT], m_pointsL[SHACKLE], m_pointsL[CLAMP_B], rs, ChColor(0.5f, 0.5f, 0.7f));
    AddVisualizationLink(m_rearleaf[RIGHT], m_pointsR[SHACKLE], m_pointsR[CLAMP_B], rs, ChColor(0.5f, 0.5f, 0.7f));

    // shackle visualisation
    AddVisualizationLink(m_shackle[LEFT], m_pointsL[SHACKLE], m_pointsL[REAR_HANGER], rs, ChColor(0.7f, 0.5f, 0.5f));
    AddVisualizationLink(m_shackle[RIGHT], m_pointsR[SHACKLE], m_pointsR[REAR_HANGER], rs, ChColor(0.7f, 0.5f, 0.5f));
}

void ChSAEToeBarLeafspringAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);
    ChPart::RemoveVisualizationAssets(m_tierod);
    ChPart::RemoveVisualizationAssets(m_draglink);

    ChPart::RemoveVisualizationAssets(m_knuckle[LEFT]);
    ChPart::RemoveVisualizationAssets(m_knuckle[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAEToeBarLeafspringAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                     const ChVector3d pt_1,
                                                     const ChVector3d pt_2,
                                                     double radius,
                                                     const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector3d p_2 = body->TransformPointParentToLocal(pt_2);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
}

void ChSAEToeBarLeafspringAxle::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                                        const ChVector3d pt_U,
                                                        const ChVector3d pt_L,
                                                        const ChVector3d pt_T,
                                                        double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector3d p_U = knuckle->TransformPointParentToLocal(pt_U);
    ChVector3d p_L = knuckle->TransformPointParentToLocal(pt_L);
    ChVector3d p_T = knuckle->TransformPointParentToLocal(pt_T);

    if (p_L.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_L, VNULL, radius);
    }

    if (p_U.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_U, VNULL, radius);
    }

    if (p_T.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_T, VNULL, radius);
    }
}
// -----------------------------------------------------------------------------

void ChSAEToeBarLeafspringAxle::PopulateComponentList() {
    m_bodies.push_back(m_spindle[0]);
    m_bodies.push_back(m_spindle[1]);
    m_bodies.push_back(m_axleTube);
    m_bodies.push_back(m_tierod);
    m_bodies.push_back(m_draglink);
    m_bodies.push_back(m_knuckle[0]);
    m_bodies.push_back(m_knuckle[1]);

    m_shafts.push_back(m_axle[0]);
    m_shafts.push_back(m_axle[1]);

    m_joints.push_back(m_frontleafSph[0]);
    m_joints.push_back(m_frontleafSph[1]);
    m_joints.push_back(m_rearleafSph[0]);
    m_joints.push_back(m_rearleafSph[1]);
    m_shackleRev[0]->IsKinematic() ? m_joints.push_back(m_shackleRev[0]->GetAsLink())
                                   : m_body_loads.push_back(m_shackleRev[0]->GetAsBushing());
    m_shackleRev[1]->IsKinematic() ? m_joints.push_back(m_shackleRev[1]->GetAsLink())
                                   : m_body_loads.push_back(m_shackleRev[1]->GetAsBushing());
    m_clampARev[0]->IsKinematic() ? m_joints.push_back(m_clampARev[0]->GetAsLink())
                                  : m_body_loads.push_back(m_clampARev[0]->GetAsBushing());
    m_clampARev[1]->IsKinematic() ? m_joints.push_back(m_clampARev[1]->GetAsLink())
                                  : m_body_loads.push_back(m_clampARev[1]->GetAsBushing());
    m_clampBRev[0]->IsKinematic() ? m_joints.push_back(m_clampBRev[0]->GetAsLink())
                                  : m_body_loads.push_back(m_clampBRev[0]->GetAsBushing());
    m_clampBRev[1]->IsKinematic() ? m_joints.push_back(m_clampBRev[1]->GetAsLink())
                                  : m_body_loads.push_back(m_clampBRev[1]->GetAsBushing());
    m_frontleafRev[0]->IsKinematic() ? m_joints.push_back(m_frontleafRev[0]->GetAsLink())
                                     : m_body_loads.push_back(m_frontleafRev[0]->GetAsBushing());
    m_frontleafRev[1]->IsKinematic() ? m_joints.push_back(m_frontleafRev[1]->GetAsLink())
                                     : m_body_loads.push_back(m_frontleafRev[1]->GetAsBushing());
    m_rearleafRev[0]->IsKinematic() ? m_joints.push_back(m_rearleafRev[0]->GetAsLink())
                                    : m_body_loads.push_back(m_rearleafRev[0]->GetAsBushing());
    m_rearleafRev[1]->IsKinematic() ? m_joints.push_back(m_rearleafRev[1]->GetAsLink())
                                    : m_body_loads.push_back(m_rearleafRev[1]->GetAsBushing());
    m_joints.push_back(m_revolute[0]);
    m_joints.push_back(m_revolute[1]);
    m_joints.push_back(m_sphericalTierod);
    m_joints.push_back(m_sphericalDraglink);
    m_joints.push_back(m_universalDraglink);
    m_joints.push_back(m_universalTierod);
    m_joints.push_back(m_revoluteKingpin[0]);
    m_joints.push_back(m_revoluteKingpin[1]);

    m_tsdas.push_back(m_spring[0]);
    m_tsdas.push_back(m_spring[1]);
    m_tsdas.push_back(m_shock[0]);
    m_tsdas.push_back(m_shock[1]);

    m_rsdas.push_back(m_latRotSpringA[0]);
    m_rsdas.push_back(m_latRotSpringA[1]);
    m_rsdas.push_back(m_latRotSpringB[0]);
    m_rsdas.push_back(m_latRotSpringB[1]);
    m_rsdas.push_back(m_vertRotSpringA[0]);
    m_rsdas.push_back(m_vertRotSpringA[1]);
    m_rsdas.push_back(m_vertRotSpringB[0]);
    m_rsdas.push_back(m_vertRotSpringB[1]);
}

}  // end namespace vehicle
}  // end namespace chrono
