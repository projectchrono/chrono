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
// Base class for a leaf-spring solid axle suspension.
//
// Base class for a leaf-spring solid axle suspension.
// Derived from ChSuspension, but still an abstract base class.
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
// body with a sperical joint. The shackle body connects to the chassis via a
// rotational joint around Y.
//
// The vertical stiffnesses are simulated by the rotational springs of the frontleaf
// and the rearleaf while the lateral stiffnesses are simulated by the rotational
// joints of the clamp bodies.
//
// For the stiffness parameters the user can take the desired translatoric vertical
// stiffness devided by two (we have two leaves). The preload can be set as a
// vertical force devided by two. The conversion to the rotary setup is made
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSAELeafspringAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSAELeafspringAxle::m_pointNames[] = {"SHOCK_A     ", "SHOCK_C     ", "SPRING_A    ", "SPRING_C    ",
                                                         "SPINDLE     ", "CLAMP_A     ", "CLAMP_B     ", "FRONT_HANGER",
                                                         "REAR_HANGER ", "SHACKLE     "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSAELeafspringAxle::ChSAELeafspringAxle(const std::string& name) : ChSuspension(name) {}

ChSAELeafspringAxle::~ChSAELeafspringAxle() {
    auto sys = m_axleTube->GetSystem();
    if (sys) {
        sys->Remove(m_axleTube);

        for (int i = 0; i < 2; i++) {
            sys->Remove(m_shock[i]);
            sys->Remove(m_spring[i]);

            sys->Remove(m_shackle[i]);
            sys->Remove(m_frontleaf[i]);
            sys->Remove(m_frontleafSph[i]);
            sys->Remove(m_rearleaf[i]);
            sys->Remove(m_rearleafSph[i]);
            sys->Remove(m_clampA[i]);
            sys->Remove(m_clampB[i]);

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
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAELeafspringAxle::Initialize(std::shared_ptr<ChChassis> chassis,
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
    ////ChVector<> midpoint_local = 0.0;
    ////ChVector<> outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    ChVector<> outer_local(getLocation(SPINDLE));
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

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, right_ang_vel);
}

void ChSAELeafspringAxle::InitializeSide(VehicleSide side,
                                         std::shared_ptr<ChChassis> chassis,
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
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrame_REF_to_abs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * Q_from_AngZ(sign * getToeAngle()) * Q_from_AngX(sign * getCamberAngle());

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize the revolute joint between axle tube and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], spindleRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_axleTube, rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis->GetBody(), m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis->GetBody(), m_axleTube, false, points[SPRING_C], points[SPRING_A]);
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

    // Leafspring related elements
    // Create and initialize the shackle body.
    m_shackle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_shackle[side]->SetNameString(m_name + "_shackle" + suffix);
    m_shackle[side]->SetPos((points[REAR_HANGER] + points[SHACKLE]) / 2.0);
    m_shackle[side]->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_shackle[side]->SetMass(getShackleMass());
    m_shackle[side]->SetInertiaXX(getShackleInertia());
    chassis->GetSystem()->AddBody(m_shackle[side]);

    // chassis-shackle rev joint
    ChCoordsys<> rev_csys_shackle(points[REAR_HANGER], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_shackleRev[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::REVOLUTE, m_name + "_shackleRev" + suffix, m_shackle[side], chassis->GetBody(),
        rev_csys_shackle, getShackleBushingData());
    chassis->AddJoint(m_shackleRev[side]);

    // Create and initialize the frontleaf body.
    m_frontleaf[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_frontleaf[side]->SetNameString(m_name + "_frontleaf" + suffix);
    m_frontleaf[side]->SetPos((points[FRONT_HANGER] + points[CLAMP_A]) / 2.0);
    m_frontleaf[side]->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_frontleaf[side]->SetMass(getFrontLeafMass());
    m_frontleaf[side]->SetInertiaXX(getFrontLeafInertia());
    chassis->GetSystem()->AddBody(m_frontleaf[side]);

    // Create and initialize the spherical joint between frontleaf and chassis
    m_frontleafSph[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_frontleafSph[side]->SetNameString(m_name + "_frontleafSpherical" + suffix);
    m_frontleafSph[side]->Initialize(m_frontleaf[side], chassis->GetBody(), ChCoordsys<>(points[FRONT_HANGER], QUNIT));
    chassis->GetSystem()->AddLink(m_frontleafSph[side]);

    // Create and initialize the rearleaf body.
    m_rearleaf[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_rearleaf[side]->SetNameString(m_name + "_rearleaf" + suffix);
    m_rearleaf[side]->SetPos((points[SHACKLE] + points[CLAMP_B]) / 2.0);
    m_rearleaf[side]->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_rearleaf[side]->SetMass(getRearLeafMass());
    m_rearleaf[side]->SetInertiaXX(getRearLeafInertia());
    chassis->GetSystem()->AddBody(m_rearleaf[side]);

    // Create and initialize the spherical joint between rearleaf and shackle
    m_rearleafSph[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_rearleafSph[side]->SetNameString(m_name + "_rearleafSpherical" + suffix);
    m_rearleafSph[side]->Initialize(m_rearleaf[side], m_shackle[side], ChCoordsys<>(points[SHACKLE], QUNIT));
    chassis->GetSystem()->AddLink(m_rearleafSph[side]);

    // Create and initialize the clampA body.
    ChVector<> mid = (points[CLAMP_A] + points[CLAMP_B]) / 2.0;
    m_clampA[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_clampA[side]->SetNameString(m_name + "_clampA" + suffix);
    m_clampA[side]->SetPos((points[CLAMP_A] + mid) / 2.0);
    m_clampA[side]->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_clampA[side]->SetMass(getClampMass());
    m_clampA[side]->SetInertiaXX(getClampInertia());
    chassis->GetSystem()->AddBody(m_clampA[side]);

    // clampA-axleTube rev joint (Z)
    ChCoordsys<> rev_csys_clampA(points[CLAMP_A], chassisRot);
    m_clampARev[side] =
        chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_clampARev" + suffix,
                                                  m_clampA[side], m_axleTube, rev_csys_clampA, getClampBushingData());
    chassis->AddJoint(m_clampARev[side]);

    m_latRotSpringA[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_latRotSpringA[side]->Initialize(m_clampA[side], m_axleTube, rev_csys_clampA);
    m_latRotSpringA[side]->RegisterTorqueFunctor(getLatTorqueFunctorA());
    chassis->GetSystem()->AddLink(m_latRotSpringA[side]);

    // Create and initialize the clampB body.
    // mid = (points[CLAMP_A] + points[CLAMP_B]) / 2.0;
    m_clampB[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_clampB[side]->SetNameString(m_name + "_clampB" + suffix);
    m_clampB[side]->SetPos((points[CLAMP_B] + mid) / 2.0);
    m_clampB[side]->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_clampB[side]->SetMass(getClampMass());
    m_clampB[side]->SetInertiaXX(getClampInertia());
    chassis->GetSystem()->AddBody(m_clampB[side]);

    // clampB-axleTube rev joint (Z)
    ChCoordsys<> rev_csys_clampB(points[CLAMP_B], chassisRot);
    m_clampBRev[side] =
        chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::REVOLUTE, m_name + "_clampBRev" + suffix,
                                                  m_clampB[side], m_axleTube, rev_csys_clampB, getClampBushingData());
    chassis->AddJoint(m_clampBRev[side]);

    m_latRotSpringB[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_latRotSpringB[side]->Initialize(m_clampB[side], m_axleTube, rev_csys_clampB);
    m_latRotSpringB[side]->RegisterTorqueFunctor(getLatTorqueFunctorB());
    chassis->GetSystem()->AddLink(m_latRotSpringB[side]);

    // clampB-rearleaf rev joint (Y)
    ChCoordsys<> rev_csys_rearleaf(points[CLAMP_B], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_rearleafRev[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::REVOLUTE, m_name + "_rearleafRev" + suffix, m_clampB[side], m_rearleaf[side],
        rev_csys_rearleaf, getLeafspringBushingData());
    chassis->AddJoint(m_rearleafRev[side]);

    m_vertRotSpringB[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_vertRotSpringB[side]->Initialize(m_clampB[side], m_rearleaf[side], rev_csys_rearleaf);
    m_vertRotSpringB[side]->RegisterTorqueFunctor(getVertTorqueFunctorB());
    chassis->GetSystem()->AddLink(m_vertRotSpringB[side]);

    // clampA-frontleaf rev joint (Y)
    ChCoordsys<> rev_csys_frontleaf(points[CLAMP_A], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_frontleafRev[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::REVOLUTE, m_name + "_frontleafRev" + suffix, m_clampA[side], m_frontleaf[side],
        rev_csys_frontleaf, getLeafspringBushingData());
    chassis->AddJoint(m_frontleafRev[side]);

    m_vertRotSpringA[side] = chrono_types::make_shared<ChLinkRSDA>();
    m_vertRotSpringA[side]->Initialize(m_clampA[side], m_frontleaf[side], rev_csys_frontleaf);
    m_vertRotSpringA[side]->RegisterTorqueFunctor(getVertTorqueFunctorA());
    chassis->GetSystem()->AddLink(m_vertRotSpringA[side]);
}

void ChSAELeafspringAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() +
             2 * (getSpindleMass() + getFrontLeafMass() + getRearLeafMass() + 2.0 * getClampMass() + getShackleMass());
}

void ChSAELeafspringAxle::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), m_spindle[LEFT]->GetMass(),
                           m_spindle[LEFT]->GetInertia());
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), m_spindle[RIGHT]->GetMass(),
                           m_spindle[RIGHT]->GetInertia());

    composite.AddComponent(m_frontleaf[LEFT]->GetFrame_COG_to_abs(), m_frontleaf[LEFT]->GetMass(),
                           m_frontleaf[LEFT]->GetInertia());
    composite.AddComponent(m_frontleaf[RIGHT]->GetFrame_COG_to_abs(), m_frontleaf[RIGHT]->GetMass(),
                           m_frontleaf[RIGHT]->GetInertia());

    composite.AddComponent(m_rearleaf[LEFT]->GetFrame_COG_to_abs(), m_rearleaf[LEFT]->GetMass(),
                           m_rearleaf[LEFT]->GetInertia());
    composite.AddComponent(m_rearleaf[RIGHT]->GetFrame_COG_to_abs(), m_rearleaf[RIGHT]->GetMass(),
                           m_rearleaf[RIGHT]->GetInertia());

    composite.AddComponent(m_clampA[LEFT]->GetFrame_COG_to_abs(), m_clampA[LEFT]->GetMass(),
                           m_clampA[LEFT]->GetInertia());
    composite.AddComponent(m_clampA[RIGHT]->GetFrame_COG_to_abs(), m_clampA[RIGHT]->GetMass(),
                           m_clampA[RIGHT]->GetInertia());

    composite.AddComponent(m_clampB[LEFT]->GetFrame_COG_to_abs(), m_clampB[LEFT]->GetMass(),
                           m_clampB[LEFT]->GetInertia());
    composite.AddComponent(m_clampB[RIGHT]->GetFrame_COG_to_abs(), m_clampB[RIGHT]->GetMass(),
                           m_clampB[RIGHT]->GetInertia());

    composite.AddComponent(m_shackle[LEFT]->GetFrame_COG_to_abs(), m_shackle[LEFT]->GetMass(),
                           m_shackle[LEFT]->GetInertia());
    composite.AddComponent(m_shackle[RIGHT]->GetFrame_COG_to_abs(), m_shackle[RIGHT]->GetMass(),
                           m_shackle[RIGHT]->GetInertia());

    composite.AddComponent(m_axleTube->GetFrame_COG_to_abs(), m_axleTube->GetMass(), m_axleTube->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChSAELeafspringAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
ChSuspension::Force ChSAELeafspringAxle::ReportSuspensionForce(VehicleSide side) const {
    ChSuspension::Force force;

    force.spring_force = m_spring[side]->GetForce();
    force.spring_length = m_spring[side]->GetLength();
    force.spring_velocity = m_spring[side]->GetVelocity();

    force.shock_force = m_shock[side]->GetForce();
    force.shock_length = m_shock[side]->GetLength();
    force.shock_velocity = m_shock[side]->GetVelocity();

    return force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAELeafspringAxle::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAELeafspringAxle::LogConstraintViolations(VehicleSide side) {
    // TODO: Update this to reflect new suspension joints
    // Revolute joints

    {}
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAELeafspringAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.03, 150, 10));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.03, 150, 10));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());

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

    // Shackle visualisation
    AddVisualizationLink(m_shackle[LEFT], m_pointsL[SHACKLE], m_pointsL[REAR_HANGER], rs, ChColor(0.7f, 0.5f, 0.5f));
    AddVisualizationLink(m_shackle[RIGHT], m_pointsR[SHACKLE], m_pointsR[REAR_HANGER], rs, ChColor(0.7f, 0.5f, 0.5f));
}

void ChSAELeafspringAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_frontleaf[LEFT]);
    ChPart::RemoveVisualizationAssets(m_frontleaf[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_rearleaf[LEFT]);
    ChPart::RemoveVisualizationAssets(m_rearleaf[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_clampA[LEFT]);
    ChPart::RemoveVisualizationAssets(m_clampA[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shackle[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shackle[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSAELeafspringAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
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
void ChSAELeafspringAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_rearleaf[0]);
    bodies.push_back(m_rearleaf[1]);
    bodies.push_back(m_frontleaf[0]);
    bodies.push_back(m_frontleaf[1]);
    bodies.push_back(m_shackle[0]);
    bodies.push_back(m_shackle[1]);
    bodies.push_back(m_clampA[0]);
    bodies.push_back(m_clampA[1]);
    bodies.push_back(m_clampB[0]);
    bodies.push_back(m_clampB[1]);
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_frontleafSph[0]);
    joints.push_back(m_frontleafSph[1]);
    joints.push_back(m_rearleafSph[0]);
    joints.push_back(m_rearleafSph[1]);
    m_shackleRev[0]->IsKinematic() ? joints.push_back(m_shackleRev[0]->GetAsLink())
                                   : bushings.push_back(m_shackleRev[0]->GetAsBushing());
    m_shackleRev[1]->IsKinematic() ? joints.push_back(m_shackleRev[1]->GetAsLink())
                                   : bushings.push_back(m_shackleRev[1]->GetAsBushing());
    m_clampARev[0]->IsKinematic() ? joints.push_back(m_clampARev[0]->GetAsLink())
                                  : bushings.push_back(m_clampARev[0]->GetAsBushing());
    m_clampARev[1]->IsKinematic() ? joints.push_back(m_clampARev[1]->GetAsLink())
                                  : bushings.push_back(m_clampARev[1]->GetAsBushing());
    m_clampBRev[0]->IsKinematic() ? joints.push_back(m_clampBRev[0]->GetAsLink())
                                  : bushings.push_back(m_clampBRev[0]->GetAsBushing());
    m_clampBRev[1]->IsKinematic() ? joints.push_back(m_clampBRev[1]->GetAsLink())
                                  : bushings.push_back(m_clampBRev[1]->GetAsBushing());
    m_frontleafRev[0]->IsKinematic() ? joints.push_back(m_frontleafRev[0]->GetAsLink())
                                     : bushings.push_back(m_frontleafRev[0]->GetAsBushing());
    m_frontleafRev[1]->IsKinematic() ? joints.push_back(m_frontleafRev[1]->GetAsLink())
                                     : bushings.push_back(m_frontleafRev[1]->GetAsBushing());
    m_rearleafRev[0]->IsKinematic() ? joints.push_back(m_rearleafRev[0]->GetAsLink())
                                    : bushings.push_back(m_rearleafRev[0]->GetAsBushing());
    m_rearleafRev[1]->IsKinematic() ? joints.push_back(m_rearleafRev[1]->GetAsLink())
                                    : bushings.push_back(m_rearleafRev[1]->GetAsBushing());
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    ChPart::ExportJointList(jsonDocument, joints);
    ChPart::ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChSAELeafspringAxle::Output(ChVehicleOutput& database) const {
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
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_frontleafSph[0]);
    joints.push_back(m_frontleafSph[1]);
    joints.push_back(m_rearleafSph[0]);
    joints.push_back(m_rearleafSph[1]);
    m_shackleRev[0]->IsKinematic() ? joints.push_back(m_shackleRev[0]->GetAsLink())
                                   : bushings.push_back(m_shackleRev[0]->GetAsBushing());
    m_shackleRev[1]->IsKinematic() ? joints.push_back(m_shackleRev[1]->GetAsLink())
                                   : bushings.push_back(m_shackleRev[1]->GetAsBushing());
    m_clampARev[0]->IsKinematic() ? joints.push_back(m_clampARev[0]->GetAsLink())
                                  : bushings.push_back(m_clampARev[0]->GetAsBushing());
    m_clampARev[1]->IsKinematic() ? joints.push_back(m_clampARev[1]->GetAsLink())
                                  : bushings.push_back(m_clampARev[1]->GetAsBushing());
    m_clampBRev[0]->IsKinematic() ? joints.push_back(m_clampBRev[0]->GetAsLink())
                                  : bushings.push_back(m_clampBRev[0]->GetAsBushing());
    m_clampBRev[1]->IsKinematic() ? joints.push_back(m_clampBRev[1]->GetAsLink())
                                  : bushings.push_back(m_clampBRev[1]->GetAsBushing());
    m_frontleafRev[0]->IsKinematic() ? joints.push_back(m_frontleafRev[0]->GetAsLink())
                                     : bushings.push_back(m_frontleafRev[0]->GetAsBushing());
    m_frontleafRev[1]->IsKinematic() ? joints.push_back(m_frontleafRev[1]->GetAsLink())
                                     : bushings.push_back(m_frontleafRev[1]->GetAsBushing());
    m_rearleafRev[0]->IsKinematic() ? joints.push_back(m_rearleafRev[0]->GetAsLink())
                                    : bushings.push_back(m_rearleafRev[0]->GetAsBushing());
    m_rearleafRev[1]->IsKinematic() ? joints.push_back(m_rearleafRev[1]->GetAsLink())
                                    : bushings.push_back(m_rearleafRev[1]->GetAsBushing());
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    database.WriteJoints(joints);
    database.WriteBodyLoads(bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
