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
// Base class for solid axle suspension with triangular and longitudinal guides.
//
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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointShape.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSolidBellcrankThreeLinkAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSolidBellcrankThreeLinkAxle::m_pointNames[] = {
    "SHOCK_A    ", "SHOCK_C    ", "SPRING_A   ", "SPRING_C   ", "SPINDLE    ", "TRIANGLE_A ",
    "TRIANGLE_C ", "LINK_A     ", "LINK_C     ", "BELLCRANK_A", "BELLCRANK_D", "BELLCRANK_T",
    "DRAGLINK_S ", "KNUCKLE_L  ", "KNUCKLE_U  ", "KNUCKLE_T  ", "KNUCKLE_CM "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSolidBellcrankThreeLinkAxle::ChSolidBellcrankThreeLinkAxle(const std::string& name) : ChSuspension(name) {}

ChSolidBellcrankThreeLinkAxle::~ChSolidBellcrankThreeLinkAxle() {
    auto sys = m_axleTube->GetSystem();
    if (sys) {
        sys->Remove(m_axleTube);
        sys->Remove(m_bellcrank);
        sys->Remove(m_draglink);

        sys->Remove(m_revBellcrank);
        sys->Remove(m_sphericalDraglink);
        sys->Remove(m_universalDraglink);

        sys->Remove(m_triangleBody);
        sys->Remove(m_triangleRev);
        sys->Remove(m_triangleSph);

        for (int i = 0; i < 2; i++) {
            sys->Remove(m_knuckle[i]);
            sys->Remove(m_revKingpin[i]);
            sys->Remove(m_linkBody[i]);
            sys->Remove(m_linkBodyToChassis[i]);
            sys->Remove(m_linkBodyToAxleTube[i]);
            sys->Remove(m_tierodBody[i]);
            sys->Remove(m_tierodBodyToKnuckle[i]);
            sys->Remove(m_tierodBodyToBellcrank[i]);

            sys->Remove(m_shock[i]);
            sys->Remove(m_spring[i]);
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidBellcrankThreeLinkAxle::Initialize(std::shared_ptr<ChChassis> chassis,
                                               std::shared_ptr<ChSubchassis> subchassis,
                                               std::shared_ptr<ChSteering> steering,
                                               const ChVector<>& location,
                                               double left_ang_vel,
                                               double right_ang_vel) {
    m_parent = chassis;
    m_rel_loc = location;

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
    ChVector<> outer_local((getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L)) / 2);
    m_axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    // Create and initialize the axle body.
    m_axleTube = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
    m_axleTube->SetNameString(m_name + "_axleTube");
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTube);

    // Transform all hardpoints to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);
    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    }

    m_tierodInnerL = m_pointsL[BELLCRANK_T];
    m_tierodInnerR = m_pointsR[BELLCRANK_T];
    m_tierodOuterL = m_pointsL[KNUCKLE_T];
    m_tierodOuterR = m_pointsR[KNUCKLE_T];

    // Fix the triangle body to the axle tube body
    ChVector<> pt_tri_axle = (m_pointsL[TRIANGLE_A] + m_pointsR[TRIANGLE_A]) / 2.0;
    ChVector<> pt_tri_chassis = (m_pointsL[TRIANGLE_C] + m_pointsR[TRIANGLE_C]) / 2.0;
    ChVector<> pt_tri_cog = (pt_tri_axle + pt_tri_chassis) / 2.0;
    m_triangle_left_point = m_pointsL[TRIANGLE_C];
    m_triangle_right_point = m_pointsR[TRIANGLE_C];
    m_triangle_sph_point = pt_tri_axle;

    // generate triangle body
    m_triangleBody = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
    m_triangleBody->SetNameString(m_name + "_triangleGuide");
    m_triangleBody->SetPos(pt_tri_cog);
    m_triangleBody->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_triangleBody->SetMass(getTriangleMass());
    m_triangleBody->SetInertiaXX(getTriangleInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_triangleBody);

    // Create and initialize the revolute joint between chassis and triangle.
    ChCoordsys<> rev_csys(pt_tri_chassis, chassis->GetBody()->GetFrame_REF_to_abs().GetRot() * Q_from_AngX(CH_C_PI_2));
    m_triangleRev = chrono_types::make_shared<ChLinkLockRevolute>();
    m_triangleRev->SetNameString(m_name + "_revoluteTriangle");
    m_triangleRev->Initialize(m_triangleBody, chassis->GetBody(), rev_csys);
    chassis->GetBody()->GetSystem()->AddLink(m_triangleRev);

    // Create and initialize the spherical joint between axle tube and triangle.
    ChCoordsys<> sph_csys(pt_tri_axle, chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_triangleSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_triangleSph->SetNameString(m_name + "_sphericalTriangle");
    m_triangleSph->Initialize(m_triangleBody, m_axleTube, sph_csys);
    chassis->GetBody()->GetSystem()->AddLink(m_triangleSph);

    m_link_axleL = m_pointsL[LINK_A];
    m_link_axleR = m_pointsR[LINK_A];
    m_link_chassisL = m_pointsL[LINK_C];
    m_link_chassisR = m_pointsR[LINK_C];

    // Initialize left and right sides.
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();
    InitializeSide(LEFT, chassis->GetBody(), tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), tierod_body, m_pointsR, right_ang_vel);
}

void ChSolidBellcrankThreeLinkAxle::InitializeSide(VehicleSide side,
                                                   std::shared_ptr<ChBodyAuxRef> chassis,
                                                   std::shared_ptr<ChBody> tierod_body,
                                                   const std::vector<ChVector<>>& points,
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

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * Q_from_AngZ(sign * getToeAngle()) * Q_from_AngX(sign * getCamberAngle());

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_knuckle[side]->SetNameString(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(spindleRot);
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

    // Create and initialize the revolute joint between axle and knuckle.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction.
    w = points[KNUCKLE_L] - points[KNUCKLE_U];
    w.Normalize();
    u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_revKingpin[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
    m_revKingpin[side]->Initialize(m_axleTube, m_knuckle[side],
                                   ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revKingpin[side]);

    // Create and initialize the revolute joint between knuckle and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], spindleRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revoluteSpindle" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_axleTube, false, points[SPRING_C], points[SPRING_A]);
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

    if (side == LEFT) {
        // create bellcrank body
        m_bellcrank = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
        m_bellcrank->SetNameString(m_name + "m_bellcrank");
        m_bellcrank->SetPos(points[BELLCRANK_A]);
        m_bellcrank->SetRot(chassis->GetRot() * ChQuaternion<>(1, 0, 0, 0));
        m_bellcrank->SetMass(getBellcrankMass());
        m_bellcrank->SetInertiaXX(getBellcrankInertia());
        chassis->GetSystem()->AddBody(m_bellcrank);

        // create bellcrank rev joint on the axleTube
        rev_csys = ChCoordsys<>(points[BELLCRANK_A], chassis->GetRot() * ChQuaternion<>(1, 0, 0, 0));
        m_revBellcrank = chrono_types::make_shared<ChLinkLockRevolute>();
        m_revBellcrank->SetNameString(m_name + "m_revBellcrank");
        m_revBellcrank->Initialize(m_axleTube, m_bellcrank, rev_csys);
        chassis->GetSystem()->AddLink(m_revBellcrank);

        // create draglink body
        m_draglink = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
        m_draglink->SetNameString(m_name + "m_draglink");
        m_draglink->SetPos((points[DRAGLINK_S] + points[BELLCRANK_D]) / 2.0);
        m_draglink->SetRot(chassis->GetRot() * ChQuaternion<>(1, 0, 0, 0));
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetSystem()->Add(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetNameString(m_name + "_sphericalDraglink");
        m_sphericalDraglink->Initialize(m_draglink, tierod_body, ChCoordsys<>(points[DRAGLINK_S], QUNIT));
        // m_sphericalDraglink->Initialize(m_draglink, chassis, ChCoordsys<>(points[DRAGLINK_C], QUNIT));
        chassis->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize the draglink body (one side only).
        // Determine the rotation matrix of the draglink based on the plane of the hard points
        // (z-axis along the length of the draglink)
        v = Vcross(points[DRAGLINK_S] - points[BELLCRANK_D], ChVector<>(0, 0, 1));
        v.Normalize();
        w = points[DRAGLINK_S] - points[BELLCRANK_D];
        w.Normalize();
        u = Vcross(v, w);
        rot.Set_A_axis(u, v, w);

        // Create and initialize the universal joint between draglink and knuckle
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetNameString(m_name + "_universalDraglink");
        m_universalDraglink->Initialize(m_draglink, m_bellcrank,
                                        ChFrame<>(points[BELLCRANK_D], rot.Get_A_quaternion()));
        chassis->GetSystem()->AddLink(m_universalDraglink);
    }

    // Create and initialize spindle body (same orientation as the chassis)
    m_linkBody[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_linkBody[side]->SetNameString(m_name + "_linkBody" + suffix);
    m_linkBody[side]->SetPos((points[LINK_A] + points[LINK_C]) / 2.0);
    m_linkBody[side]->SetRot(chassisRot);
    m_linkBody[side]->SetMass(getLinkMass());
    m_linkBody[side]->SetInertiaXX(getLinkInertia());
    chassis->GetSystem()->AddBody(m_linkBody[side]);

    // Create and initialize the spherical joint between axle tube and link body.
    ChCoordsys<> sph1_csys(points[LINK_A], chassisRot);
    m_linkBodyToAxleTube[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_linkBodyToAxleTube[side]->SetNameString(m_name + "_sphericalLinkToAxle" + suffix);
    m_linkBodyToAxleTube[side]->Initialize(m_linkBody[side], m_axleTube, sph1_csys);
    chassis->GetSystem()->AddLink(m_linkBodyToAxleTube[side]);

    // Create and initialize the spherical joint between chassis and link body.
    v = Vcross(points[LINK_C] - points[LINK_A], ChVector<>(0, 1, 0));
    v.Normalize();
    w = points[LINK_C] - points[LINK_A];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    // Create and initialize the universal joint between draglink and knuckle
    m_linkBodyToChassis[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_linkBodyToChassis[side]->SetNameString(m_name + "_universalDraglink");
    m_linkBodyToChassis[side]->Initialize(m_linkBody[side], chassis, ChFrame<>(points[LINK_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_linkBodyToChassis[side]);

    // Create the tierod as rigid body
    m_tierodBody[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_tierodBody[side]->SetNameString(m_name + "_tierodBody" + suffix);
    m_tierodBody[side]->SetPos((points[KNUCKLE_T] + points[BELLCRANK_T]) / 2.0);
    m_tierodBody[side]->SetRot(chassisRot);
    m_tierodBody[side]->SetMass(getTierodMass());
    m_tierodBody[side]->SetInertiaXX(getTierodInertia());
    chassis->GetSystem()->AddBody(m_tierodBody[side]);

    // Connect tierod to knuckle via spherical link
    ChCoordsys<> sph3_csys(points[KNUCKLE_T], chassisRot);
    m_tierodBodyToKnuckle[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_tierodBodyToKnuckle[side]->SetNameString(m_name + "_tierodLinkToKnuckle" + suffix);
    m_tierodBodyToKnuckle[side]->Initialize(m_tierodBody[side], m_knuckle[side], sph3_csys);
    chassis->GetSystem()->AddLink(m_tierodBodyToKnuckle[side]);

    // Connect tierod to bellcrank via spherical link
    ChCoordsys<> sph4_csys(points[BELLCRANK_T], chassisRot);
    m_tierodBodyToBellcrank[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_tierodBodyToBellcrank[side]->SetNameString(m_name + "_tierodLinkToBellcrank" + suffix);
    m_tierodBodyToBellcrank[side]->Initialize(m_tierodBody[side], m_bellcrank, sph4_csys);
    chassis->GetSystem()->AddLink(m_tierodBodyToBellcrank[side]);
}

void ChSolidBellcrankThreeLinkAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + getTriangleMass() + getBellcrankMass() + getDraglinkMass() +
             2 * (getSpindleMass() + getKnuckleMass() + getTierodMass() + getLinkMass());
}

void ChSolidBellcrankThreeLinkAxle::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());
    ChMatrix33<> inertiaLink(getLinkInertia());
    ChMatrix33<> inertiaTierod(getTierodInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);

    composite.AddComponent(m_knuckle[LEFT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckle[RIGHT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);

    composite.AddComponent(m_linkBody[LEFT]->GetFrame_COG_to_abs(), getLinkMass(), inertiaLink);
    composite.AddComponent(m_linkBody[RIGHT]->GetFrame_COG_to_abs(), getLinkMass(), inertiaLink);

    composite.AddComponent(m_tierodBody[LEFT]->GetFrame_COG_to_abs(), getTierodMass(), inertiaTierod);
    composite.AddComponent(m_tierodBody[RIGHT]->GetFrame_COG_to_abs(), getTierodMass(), inertiaTierod);

    composite.AddComponent(m_axleTube->GetFrame_COG_to_abs(), getAxleTubeMass(), ChMatrix33<>(getAxleTubeInertia()));
    composite.AddComponent(m_bellcrank->GetFrame_COG_to_abs(), getBellcrankMass(), ChMatrix33<>(getBellcrankInertia()));
    composite.AddComponent(m_draglink->GetFrame_COG_to_abs(), getDraglinkMass(), ChMatrix33<>(getDraglinkInertia()));
    composite.AddComponent(m_triangleBody->GetFrame_COG_to_abs(), getTriangleMass(),
                           ChMatrix33<>(getTriangleInertia()));

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChSolidBellcrankThreeLinkAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChSolidBellcrankThreeLinkAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidBellcrankThreeLinkAxle::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidBellcrankThreeLinkAxle::LogConstraintViolations(VehicleSide side) {
    // TODO: Update this to reflect new suspension joints
    // Revolute joints

    {}
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidBellcrankThreeLinkAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // visualize the axle tube
    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));

    // visualize the trangle body
    AddVisualizationLink(m_triangleBody, m_triangle_sph_point, m_triangle_left_point, getAxleTubeRadius() / 2.0,
                         ChColor(0.7f, 0.3f, 0.8f));
    AddVisualizationLink(m_triangleBody, m_triangle_sph_point, m_triangle_right_point, getAxleTubeRadius() / 2.0,
                         ChColor(0.7f, 0.3f, 0.8f));

    // visualize the trangle body
    AddVisualizationLink(m_linkBody[LEFT], m_link_axleL, m_link_chassisL, getAxleTubeRadius() / 2.0,
                         ChColor(0.3f, 0.3f, 0.8f));
    AddVisualizationLink(m_linkBody[RIGHT], m_link_axleR, m_link_chassisR, getAxleTubeRadius() / 2.0,
                         ChColor(0.3f, 0.3f, 0.8f));

    // visualize the draglink
    AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_S], m_pointsL[BELLCRANK_D], 0.02, ChColor(0.3f, 0.3f, 0.3f));

    // visualize the bellcrank base
    AddVisualizationLink(m_bellcrank, m_pointsL[BELLCRANK_A] - ChVector<>(0, 0, 0.1),
                         m_pointsL[BELLCRANK_A] + ChVector<>(0, 0, 0.1), 0.05, ChColor(0.7f, 0.3f, 0.7f));
    // visualize the bellcrank drag arm
    AddVisualizationLink(m_bellcrank, m_pointsL[BELLCRANK_A], m_pointsL[BELLCRANK_D], 0.02, ChColor(0.7f, 0.3f, 0.7f));
    // visualize the bellcrank tierod arms
    AddVisualizationLink(m_bellcrank, m_pointsL[BELLCRANK_A], m_pointsL[BELLCRANK_T], 0.02, ChColor(0.7f, 0.3f, 0.7f));
    AddVisualizationLink(m_bellcrank, m_pointsL[BELLCRANK_A], m_pointsR[BELLCRANK_T], 0.02, ChColor(0.7f, 0.3f, 0.7f));

    // visualize the knuckles, kingpin
    AddVisualizationLink(m_knuckle[LEFT], m_pointsL[KNUCKLE_L], m_pointsL[KNUCKLE_U], 0.03, ChColor(0.7f, 0.3f, 0.3f));
    AddVisualizationLink(m_knuckle[RIGHT], m_pointsR[KNUCKLE_L], m_pointsR[KNUCKLE_U], 0.03, ChColor(0.7f, 0.3f, 0.3f));

    // visualize the knuckles, upright
    AddVisualizationLink(m_knuckle[LEFT], m_axleOuterL, m_pointsL[SPINDLE], 0.03, ChColor(0.7f, 0.3f, 0.3f));
    AddVisualizationLink(m_knuckle[RIGHT], m_axleOuterR, m_pointsR[SPINDLE], 0.03, ChColor(0.7f, 0.3f, 0.3f));

    // visualize the knuckles, steering arms
    AddVisualizationLink(m_knuckle[LEFT], m_axleOuterL, m_pointsL[KNUCKLE_T], 0.03, ChColor(0.7f, 0.3f, 0.3f));
    AddVisualizationLink(m_knuckle[RIGHT], m_axleOuterR, m_pointsR[KNUCKLE_T], 0.03, ChColor(0.7f, 0.3f, 0.3f));

    // visualize the tierods
    AddVisualizationLink(m_tierodBody[LEFT], m_tierodOuterL, m_tierodInnerL, 0.03, ChColor(0.3f, 0.7f, 0.3f));
    AddVisualizationLink(m_tierodBody[RIGHT], m_tierodOuterR, m_tierodInnerR, 0.03, ChColor(0.3f, 0.7f, 0.3f));

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
}

void ChSolidBellcrankThreeLinkAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidBellcrankThreeLinkAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                         const ChVector<> pt_1,
                                                         const ChVector<> pt_2,
                                                         double radius,
                                                         const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_1;
    cyl->GetCylinderGeometry().p2 = p_2;
    cyl->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSolidBellcrankThreeLinkAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChSolidBellcrankThreeLinkAxle::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // namespace vehicle
}  // end namespace chrono
