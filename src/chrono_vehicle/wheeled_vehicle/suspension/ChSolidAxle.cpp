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
// suspension and will be mirrored (reflecting the y coordinates) to construct
// the right side.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSolidAxle::m_pointNames[] = {
    "SHOCK_A    ", "SHOCK_C    ", "KNUCKLE_L  ", "KNUCKLE_U  ", "LL_A       ", "LL_C       ",
    "UL_A       ", "UL_C       ", "SPRING_A   ", "SPRING_C   ", "TIEROD_C   ", "TIEROD_K   ",
    "SPINDLE    ", "KNUCKLE_CM ", "LL_CM      ", "UL_CM      ", "TRACKBAR_A ", "TRACKBAR_C "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSolidAxle::ChSolidAxle(const std::string& name) : ChSuspension(name) {}

ChSolidAxle::~ChSolidAxle() {
    if (!m_initialized)
        return;

    auto sys = m_axleTube->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_axleTube);
    sys->Remove(m_tierod);
    sys->Remove(m_bellCrank);
    sys->Remove(m_draglink);
    sys->Remove(m_trackbarBody);

    sys->Remove(m_revoluteBellCrank);
    sys->Remove(m_sphericalTierod);
    sys->Remove(m_sphericalDraglink);
    sys->Remove(m_universalDraglink);
    sys->Remove(m_universalTierod);
    sys->Remove(m_pointPlaneBellCrank);
    sys->Remove(m_sphericalTrackbarAxleLink);
    sys->Remove(m_sphericalTrackbarChassisLink);

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_knuckle[i]);
        sys->Remove(m_upperLink[i]);
        sys->Remove(m_lowerLink[i]);

        sys->Remove(m_revoluteKingpin[i]);
        sys->Remove(m_sphericalUpperLink[i]);
        sys->Remove(m_sphericalLowerLink[i]);
        sys->Remove(m_universalUpperLink[i]);
        sys->Remove(m_universalLowerLink[i]);

        sys->Remove(m_shock[i]);
        sys->Remove(m_spring[i]);
    }
}

// -----------------------------------------------------------------------------
void ChSolidAxle::Construct(std::shared_ptr<ChChassis> chassis,
                            std::shared_ptr<ChSubchassis> subchassis,
                            std::shared_ptr<ChSteering> steering,
                            const ChVector3d& location,
                            double left_ang_vel,
                            double right_ang_vel) {
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

    // Create track connection points and COM
    ChVector3d trackbarAxle_local(getLocation(TRACKBAR_A));
    m_trackbarAxle = suspension_to_abs.TransformPointLocalToParent(trackbarAxle_local);
    ChVector3d trackbarChassis_local(getLocation(TRACKBAR_C));
    m_trackbarChassis = suspension_to_abs.TransformPointLocalToParent(trackbarChassis_local);

    // Create and initialize the trackbar body
    m_trackbarBody = chrono_types::make_shared<ChBody>();
    m_trackbarBody->SetName(m_name + "_trackBar");
    m_trackbarBody->SetTag(m_obj_tag);
    m_trackbarBody->SetPos((m_trackbarAxle + m_trackbarChassis) / 2);
    m_trackbarBody->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_trackbarBody->SetMass(getTrackbarMass());
    m_trackbarBody->SetInertiaXX(getTrackbarInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_trackbarBody);

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

    // Connect trackbarBody to axleBody (should be changed to universal)
    m_sphericalTrackbarAxleLink = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalTrackbarAxleLink->SetName(m_name + "_sphericalTrackbarAxle");
    m_sphericalTrackbarAxleLink->SetTag(m_obj_tag);
    m_sphericalTrackbarAxleLink->Initialize(m_trackbarBody, m_axleTube, ChFrame<>(m_trackbarAxle, QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTrackbarAxleLink);

    // Connect trackbarBody to chassisBody
    m_sphericalTrackbarChassisLink = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalTrackbarChassisLink->SetName(m_name + "_sphericalTrackbarChassis");
    m_sphericalTrackbarChassisLink->SetTag(m_obj_tag);
    m_sphericalTrackbarChassisLink->Initialize(m_trackbarBody, chassis->GetBody(), ChFrame<>(m_trackbarChassis, QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTrackbarChassisLink);

    // Initialize left and right sides.
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();
    InitializeSide(LEFT, chassis->GetBody(), tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), tierod_body, m_pointsR, right_ang_vel);
}

void ChSolidAxle::InitializeSide(VehicleSide side,
                                 std::shared_ptr<ChBodyAuxRef> chassis,
                                 std::shared_ptr<ChBody> tierod_body,
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
    ChQuaternion<> chassisRot = chassis->GetFrameRefToAbs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * QuatFromAngleZ(sign * getToeAngle()) * QuatFromAngleX(sign * getCamberAngle());

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = chrono_types::make_shared<ChBody>();
    m_knuckle[side]->SetName(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetTag(m_obj_tag);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(spindleRot);
    m_knuckle[side]->SetMass(getKnuckleMass());
    m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
    chassis->GetSystem()->AddBody(m_knuckle[side]);

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = chrono_types::make_shared<ChBody>();
    m_spindle[side]->SetName(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
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
    rot.SetFromDirectionAxes(u, v, w);

    m_upperLink[side] = chrono_types::make_shared<ChBody>();
    m_upperLink[side]->SetName(m_name + "_upperLink" + suffix);
    m_upperLink[side]->SetTag(m_obj_tag);
    m_upperLink[side]->SetPos(points[UL_CM]);
    m_upperLink[side]->SetRot(rot);
    m_upperLink[side]->SetMass(getULMass());
    m_upperLink[side]->SetInertiaXX(getULInertia());
    chassis->GetSystem()->AddBody(m_upperLink[side]);

    // Create and initialize the universal joint between chassis and upper link.
    m_universalUpperLink[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalUpperLink[side]->SetName(m_name + "_universalUpperLink" + suffix);
    m_universalUpperLink[side]->SetTag(m_obj_tag);
    m_universalUpperLink[side]->Initialize(chassis, m_upperLink[side], ChFrame<>(points[UL_C], rot.GetQuaternion()));
    chassis->GetSystem()->AddLink(m_universalUpperLink[side]);

    // Create and initialize lower link body.
    // Determine the rotation matrix of the lower link based on the plane of the hard points
    // (z-axis along the length of the lower link)
    v = Vcross(points[LL_C] - points[UL_A], points[LL_A] - points[UL_A]);
    v.Normalize();
    w = points[LL_A] - points[LL_C];
    w.Normalize();
    u = Vcross(v, w);
    rot.SetFromDirectionAxes(u, v, w);

    m_lowerLink[side] = chrono_types::make_shared<ChBody>();
    m_lowerLink[side]->SetName(m_name + "_lowerLink" + suffix);
    m_lowerLink[side]->SetTag(m_obj_tag);
    m_lowerLink[side]->SetPos(points[LL_CM]);
    m_lowerLink[side]->SetRot(rot);
    m_lowerLink[side]->SetMass(getLLMass());
    m_lowerLink[side]->SetInertiaXX(getLLInertia());
    chassis->GetSystem()->AddBody(m_lowerLink[side]);

    // Create and initialize the universal joint between chassis and lower link.
    m_universalLowerLink[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalLowerLink[side]->SetName(m_name + "_universalLowerLink" + suffix);
    m_universalLowerLink[side]->SetTag(m_obj_tag);
    m_universalLowerLink[side]->Initialize(chassis, m_lowerLink[side], ChFrame<>(points[LL_C], rot.GetQuaternion()));
    chassis->GetSystem()->AddLink(m_universalLowerLink[side]);

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

    // Create and initialize the spherical joint between axle and upper link.
    m_sphericalUpperLink[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalUpperLink[side]->SetName(m_name + "_sphericalUpperLink" + suffix);
    m_sphericalUpperLink[side]->SetTag(m_obj_tag);
    m_sphericalUpperLink[side]->Initialize(m_axleTube, m_upperLink[side], ChFrame<>(points[UL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUpperLink[side]);

    // Create and initialize the spherical joint between axle and lower link.
    m_sphericalLowerLink[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalLowerLink[side]->SetName(m_name + "_sphericalLowerLink" + suffix);
    m_sphericalLowerLink[side]->SetTag(m_obj_tag);
    m_sphericalLowerLink[side]->Initialize(m_axleTube, m_lowerLink[side], ChFrame<>(points[LL_A], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLowerLink[side]);

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
    m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetName(m_name + "_spring" + suffix);
    m_spring[side]->SetTag(m_obj_tag);
    m_spring[side]->Initialize(chassis, m_axleTube, false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
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
        rot.SetFromDirectionAxes(u, v, w);

        m_draglink = chrono_types::make_shared<ChBody>();
        m_draglink->SetName(m_name + "_draglink");
        m_draglink->SetTag(m_obj_tag);
        m_draglink->SetPos((points[DRAGLINK_C] + points[BELLCRANK_DRAGLINK]) / 2);
        m_draglink->SetRot(rot.GetQuaternion());
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetSystem()->AddBody(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetName(m_name + "_sphericalDraglink" + suffix);
        m_sphericalDraglink->SetTag(m_obj_tag);
        m_sphericalDraglink->Initialize(m_draglink, tierod_body, ChFrame<>(points[DRAGLINK_C], QUNIT));
        chassis->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize bell crank body (one side only).
        m_bellCrank = chrono_types::make_shared<ChBody>();
        m_bellCrank->SetName(m_name + "_bellCrank");
        m_bellCrank->SetTag(m_obj_tag);
        m_bellCrank->SetPos((points[BELLCRANK_DRAGLINK] + points[BELLCRANK_AXLE] + points[BELLCRANK_TIEROD]) * CH_1_3);
        m_bellCrank->SetRot(rot.GetQuaternion());
        m_bellCrank->SetMass(getBellCrankMass());
        m_bellCrank->SetInertiaXX(getBellCrankInertia());
        chassis->GetSystem()->AddBody(m_bellCrank);

        // Create and initialize the universal joint between draglink and bell crank.
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetName(m_name + "_universalDraglink" + suffix);
        m_universalDraglink->SetTag(m_obj_tag);
        m_universalDraglink->Initialize(m_draglink, m_bellCrank,
                                        ChFrame<>(points[BELLCRANK_DRAGLINK], rot.GetQuaternion()));
        chassis->GetSystem()->AddLink(m_universalDraglink);

        // Create and initialize the revolute joint between bellCrank and axle tube.
        // Determine the joint orientation matrix from the hardpoint locations by
        // constructing a rotation matrix with the z axis along the joint direction.
        w = Vcross(points[BELLCRANK_DRAGLINK] - points[BELLCRANK_AXLE],
                   points[BELLCRANK_TIEROD] - points[BELLCRANK_AXLE]);
        w.Normalize();
        v = points[BELLCRANK_TIEROD] - points[BELLCRANK_DRAGLINK];
        v.Normalize();
        u = Vcross(v, w);
        rot.SetFromDirectionAxes(u, v, w);

        m_revoluteBellCrank = chrono_types::make_shared<ChLinkLockRevolute>();
        m_revoluteBellCrank->SetName(m_name + "_revoluteBellCrank" + suffix);
        m_revoluteBellCrank->SetTag(m_obj_tag);
        m_revoluteBellCrank->Initialize(m_bellCrank, m_axleTube, ChFrame<>(points[BELLCRANK_AXLE], QUNIT));
        chassis->GetSystem()->AddLink(m_revoluteBellCrank);

        // Create and initialize the point-plane joint between bell crank and tierod.
        m_pointPlaneBellCrank = chrono_types::make_shared<ChLinkLockPointPlane>();
        m_pointPlaneBellCrank->SetName(m_name + "_pointPlaneBellCrank" + suffix);
        m_pointPlaneBellCrank->SetTag(m_obj_tag);
        m_pointPlaneBellCrank->Initialize(m_bellCrank, m_tierod,
                                          ChFrame<>(points[BELLCRANK_TIEROD], chassisRot * QuatFromAngleX(CH_PI_2)));
        chassis->GetSystem()->AddLink(m_pointPlaneBellCrank);
    }

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
}

void ChSolidAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + getTierodMass() + getDraglinkMass() + getBellCrankMass() +
             2 * (getSpindleMass() + getULMass() + getLLMass() + getKnuckleMass());
}

void ChSolidAxle::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());
    ChMatrix33<> inertiaUL(getULInertia());
    ChMatrix33<> inertiaLL(getLLInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);

    composite.AddComponent(m_upperLink[LEFT]->GetFrameCOMToAbs(), getULMass(), inertiaUL);
    composite.AddComponent(m_upperLink[RIGHT]->GetFrameCOMToAbs(), getULMass(), inertiaUL);

    composite.AddComponent(m_lowerLink[LEFT]->GetFrameCOMToAbs(), getLLMass(), inertiaLL);
    composite.AddComponent(m_lowerLink[RIGHT]->GetFrameCOMToAbs(), getLLMass(), inertiaLL);

    composite.AddComponent(m_knuckle[LEFT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckle[RIGHT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);

    composite.AddComponent(m_axleTube->GetFrameCOMToAbs(), getAxleTubeMass(), ChMatrix33<>(getAxleTubeInertia()));
    composite.AddComponent(m_tierod->GetFrameCOMToAbs(), getTierodMass(), ChMatrix33<>(getTierodInertia()));
    composite.AddComponent(m_draglink->GetFrameCOMToAbs(), getDraglinkMass(), ChMatrix33<>(getDraglinkInertia()));
    composite.AddComponent(m_bellCrank->GetFrameCOMToAbs(), getBellCrankMass(), ChMatrix33<>(getBellCrankInertia()));
    composite.AddComponent(m_trackbarBody->GetFrameCOMToAbs(), getTrackbarMass(), ChMatrix33<>(getTrackbarInertia()));

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChSolidAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChSolidAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::LogConstraintViolations(VehicleSide side) {
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
        ChVectorDynamic<> C = m_revoluteBellCrank->GetConstraintViolation();
        std::cout << "Bell Crank revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    // Spherical joints
    {
        ChVectorDynamic<> C = m_sphericalUpperLink[side]->GetConstraintViolation();
        std::cout << "UL spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalLowerLink[side]->GetConstraintViolation();
        std::cout << "LL spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
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

    // Universal joints
    {
        ChVectorDynamic<> C = m_universalUpperLink[side]->GetConstraintViolation();
        std::cout << "UL universal          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalLowerLink[side]->GetConstraintViolation();
        std::cout << "LL universal          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
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

    // Point-plane joints
    {
        ChVectorDynamic<> C = m_pointPlaneBellCrank->GetConstraintViolation();
        std::cout << "Bell Crank point-plane          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "  ";
        std::cout << "  " << C(5) << "\n";
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

    AddVisualizationLink(m_trackbarBody, m_trackbarAxle, m_trackbarChassis, getTrackbarRadius(),
                         ChColor(0.7f, 0.2f, 0.7f));

    AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius(), ChColor(0.2f, 0.7f, 0.2f));
    AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius(), ChColor(0.2f, 0.7f, 0.2f));

    AddVisualizationLink(m_upperLink[LEFT], m_pointsL[UL_A], m_pointsL[UL_C], getULRadius(), ChColor(0.6f, 0.2f, 0.6f));
    AddVisualizationLink(m_upperLink[RIGHT], m_pointsR[UL_A], m_pointsR[UL_C], getULRadius(),
                         ChColor(0.6f, 0.2f, 0.6f));

    AddVisualizationLink(m_lowerLink[LEFT], m_pointsL[LL_A], m_pointsL[LL_C], getLLRadius(), ChColor(0.2f, 0.6f, 0.2f));
    AddVisualizationLink(m_lowerLink[RIGHT], m_pointsR[LL_A], m_pointsR[LL_C], getLLRadius(),
                         ChColor(0.2f, 0.6f, 0.2f));

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChSolidAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);
    ChPart::RemoveVisualizationAssets(m_tierod);
    ChPart::RemoveVisualizationAssets(m_draglink);
    ChPart::RemoveVisualizationAssets(m_bellCrank);
    ChPart::RemoveVisualizationAssets(m_trackbarBody);

    ChPart::RemoveVisualizationAssets(m_knuckle[LEFT]);
    ChPart::RemoveVisualizationAssets(m_knuckle[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_upperLink[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upperLink[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_lowerLink[LEFT]);
    ChPart::RemoveVisualizationAssets(m_lowerLink[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                       const ChVector3d pt_1,
                                       const ChVector3d pt_2,
                                       double radius,
                                       const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector3d p_2 = body->TransformPointParentToLocal(pt_2);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
    auto ns = body->GetVisualModel()->GetNumShapes();
    for (unsigned int i = 0; i < ns; i++)
        body->GetVisualModel()->GetShape(i)->SetColor(color);
}

void ChSolidAxle::AddVisualizationBellCrank(std::shared_ptr<ChBody> body,
                                            const ChVector3d pt_D,
                                            const ChVector3d pt_A,
                                            const ChVector3d pt_T,
                                            double radius,
                                            const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_D = body->TransformPointParentToLocal(pt_D);
    ChVector3d p_A = body->TransformPointParentToLocal(pt_A);
    ChVector3d p_T = body->TransformPointParentToLocal(pt_T);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_D, p_A, radius);
    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_A, p_T, radius);

    auto ns = body->GetVisualModel()->GetNumShapes();
    for (unsigned int i = 0; i < ns; i++)
        body->GetVisualModel()->GetShape(i)->SetColor(color);
}

void ChSolidAxle::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                          const ChVector3d pt_U,
                                          const ChVector3d pt_L,
                                          const ChVector3d pt_T,
                                          double radius,
                                          const ChColor& color) {
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

    auto ns = knuckle->GetVisualModel()->GetNumShapes();
    for (unsigned int i = 0; i < ns; i++)
        knuckle->GetVisualModel()->GetShape(i)->SetColor(color);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    bodies.push_back(m_tierod);
    bodies.push_back(m_bellCrank);
    bodies.push_back(m_draglink);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    bodies.push_back(m_upperLink[0]);
    bodies.push_back(m_upperLink[1]);
    bodies.push_back(m_lowerLink[0]);
    bodies.push_back(m_lowerLink[1]);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteBellCrank);
    joints.push_back(m_sphericalTierod);
    joints.push_back(m_sphericalDraglink);
    joints.push_back(m_universalDraglink);
    joints.push_back(m_universalTierod);
    joints.push_back(m_pointPlaneBellCrank);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    joints.push_back(m_sphericalUpperLink[0]);
    joints.push_back(m_sphericalUpperLink[1]);
    joints.push_back(m_sphericalLowerLink[0]);
    joints.push_back(m_sphericalLowerLink[1]);
    joints.push_back(m_universalUpperLink[0]);
    joints.push_back(m_universalUpperLink[1]);
    joints.push_back(m_universalLowerLink[0]);
    joints.push_back(m_universalLowerLink[1]);
    ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChSolidAxle::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    bodies.push_back(m_tierod);
    bodies.push_back(m_bellCrank);
    bodies.push_back(m_draglink);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    bodies.push_back(m_upperLink[0]);
    bodies.push_back(m_upperLink[1]);
    bodies.push_back(m_lowerLink[0]);
    bodies.push_back(m_lowerLink[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_revoluteBellCrank);
    joints.push_back(m_sphericalTierod);
    joints.push_back(m_sphericalDraglink);
    joints.push_back(m_universalDraglink);
    joints.push_back(m_universalTierod);
    joints.push_back(m_pointPlaneBellCrank);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    joints.push_back(m_sphericalUpperLink[0]);
    joints.push_back(m_sphericalUpperLink[1]);
    joints.push_back(m_sphericalLowerLink[0]);
    joints.push_back(m_sphericalLowerLink[1]);
    joints.push_back(m_universalUpperLink[0]);
    joints.push_back(m_universalUpperLink[1]);
    joints.push_back(m_universalLowerLink[0]);
    joints.push_back(m_universalLowerLink[1]);
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
