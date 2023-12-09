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
// Authors: Rainer Gericke
// =============================================================================
//
// Model of a solid axle with coils springs and a Panhard rod as lateral guide
// A spherical joint acts as longitudinal guide
//
// Used as rear suspension system in Mercedes Unimogs (high mobility series)
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChPushPipeAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChPushPipeAxle::m_pointNames[] = {"SHOCK_A    ", "SHOCK_C    ", "SPRING_A   ", "SPRING_C   ",
                                                    "SPINDLE    ", "AXLE_C     ", "PANHARD_A  ", "PANHARD_C  "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChPushPipeAxle::ChPushPipeAxle(const std::string& name) : ChSuspension(name) {}

ChPushPipeAxle::~ChPushPipeAxle() {
    auto sys = m_axleTube->GetSystem();
    if (sys) {
        sys->Remove(m_axleTube);
        sys->Remove(m_panhardRod);
        for (int i = 0; i < 2; i++) {
            sys->Remove(m_shock[i]);
            sys->Remove(m_spring[i]);
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPushPipeAxle::Initialize(std::shared_ptr<ChChassis> chassis,
                                std::shared_ptr<ChSubchassis> subchassis,
                                std::shared_ptr<ChSteering> steering,
                                const ChVector<>& location,
                                double left_ang_vel,
                                double right_ang_vel) {
    ChSuspension::Initialize(chassis, subchassis, steering, location, left_ang_vel, right_ang_vel);

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
    // this is a portal axle, there is a vertical offset of the outer axle points
    ChVector<> outer_local(getLocation(SPINDLE) + ChVector<>(0, 0, getPortalOffset()));
    m_axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    m_pushPipeOuterC = suspension_to_abs.TransformPointLocalToParent(getLocation(AXLE_C));
    ChVector<> hlp = getLocation(SPINDLE) + ChVector<>(0, 0, getPortalOffset());
    ChVector<> hlp2 = hlp;
    hlp2.y() = -hlp.y();
    hlp = 0.5 * (hlp + hlp2);
    m_pushPipeOuterA = suspension_to_abs.TransformPointLocalToParent(hlp + ChVector<>(0, getLocation(AXLE_C).y(), 0));

    m_panhardOuterA = suspension_to_abs.TransformPointLocalToParent(getLocation(PANHARD_A));
    m_panhardOuterC = suspension_to_abs.TransformPointLocalToParent(getLocation(PANHARD_C));

    // Create and initialize the axle body.
    m_axleTube = chrono_types::make_shared<ChBody>();
    m_axleTube->SetNameString(m_name + "_axleTube");
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTube);

    m_axleTubeGuideLong = chrono_types::make_shared<ChLinkLockSpherical>();
    m_axleTubeGuideLong->SetNameString(m_name + "_sphereAxleTube");
    ChVector<> spPos = suspension_to_abs.TransformPointLocalToParent(getLocation(AXLE_C));
    m_axleTubeGuideLong->Initialize(m_axleTube, chassis->GetBody(), ChCoordsys<>(spPos, QUNIT));
    chassis->GetBody()->GetSystem()->AddLink(m_axleTubeGuideLong);

    // Create and initialize the panhard rod body
    ChVector<> panCom_local = 0.5 * (getLocation(PANHARD_A) + getLocation(PANHARD_C));
    ChVector<> panCom = suspension_to_abs.TransformLocalToParent(panCom_local);
    m_panhardRod = chrono_types::make_shared<ChBody>();
    m_panhardRod->SetNameString(m_name + "_panhardRod");
    m_panhardRod->SetPos(panCom);
    m_panhardRod->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_panhardRod->SetMass(getPanhardRodMass());
    m_panhardRod->SetInertiaXX(getPanhardRodInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_panhardRod);

    // fix panhard rod body to the axleTube
    ChVector<> panAxl = suspension_to_abs.TransformLocalToParent(getLocation(PANHARD_A));
    m_panhardRod2Axle = chrono_types::make_shared<ChLinkLockSpherical>();
    m_panhardRod2Axle->SetNameString(m_name + "_panhardRod2Axle");
    m_panhardRod2Axle->Initialize(m_panhardRod, m_axleTube, ChCoordsys<>(panAxl, QUNIT));
    chassis->GetBody()->GetSystem()->AddLink(m_panhardRod2Axle);

    // fix panhard rod body to the chassis body
    ChVector<> panCh = suspension_to_abs.TransformLocalToParent(getLocation(PANHARD_C));
    m_panhardRod2Chassis = chrono_types::make_shared<ChLinkLockSpherical>();
    m_panhardRod2Chassis->SetNameString(m_name + "_panhardRod2Chassis");
    m_panhardRod2Chassis->Initialize(m_panhardRod, chassis->GetBody(), ChCoordsys<>(panCh, QUNIT));
    chassis->GetBody()->GetSystem()->AddLink(m_panhardRod2Chassis);

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
    std::shared_ptr<ChBody> scbeamL = (subchassis == nullptr) ? chassis->GetBody() : subchassis->GetBeam(LEFT);
    std::shared_ptr<ChBody> scbeamR = (subchassis == nullptr) ? chassis->GetBody() : subchassis->GetBeam(RIGHT);
    InitializeSide(LEFT, chassis->GetBody(), scbeamL, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), scbeamR, m_pointsR, right_ang_vel);
}

void ChPushPipeAxle::InitializeSide(VehicleSide side,
                                    std::shared_ptr<ChBodyAuxRef> chassis,
                                    std::shared_ptr<ChBody> scbeam,
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

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = chrono_types::make_shared<ChBody>();
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

    // Create and initialize the shock damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    // Create and initialize the spring
    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(scbeam, m_axleTube, false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle.
    // Note that the spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetNameString(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPos_dt(-ang_vel);
    chassis->GetSystem()->AddShaft(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftsBody>();
    m_axle_to_spindle[side]->SetNameString(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

void ChPushPipeAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + 2 * (getSpindleMass());
}

void ChPushPipeAxle::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_axleTube->GetFrame_COG_to_abs(), getAxleTubeMass(), ChMatrix33<>(getAxleInertia()));

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChPushPipeAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChPushPipeAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPushPipeAxle::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPushPipeAxle::LogConstraintViolations(VehicleSide side) {
    {
        /*
      ChVectorDynamic<> C = m_axleTubeGuideLat->GetConstraintViolation();
      GetLog() << "Axle tube prismatic       ";
      GetLog() << "  " << C(0) << "  ";
      GetLog() << "  " << C(1) << "  ";
      GetLog() << "  " << C(2) << "\n";
         */
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPushPipeAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_axleTube, m_pushPipeOuterA, m_pushPipeOuterC, getAxleTubeRadius(),
                         ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_panhardRod, m_panhardOuterA, m_panhardOuterC, getPanhardRodRadius(),
                         ChColor(0.5f, 0.7f, 0.9f));

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChPushPipeAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);

    ChPart::RemoveVisualizationAssets(m_panhardRod);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPushPipeAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
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

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChPushPipeAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    bodies.push_back(m_panhardRod);
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_axleTubeGuideLong);
    joints.push_back(m_panhardRod2Axle);
    joints.push_back(m_panhardRod2Chassis);
    ChPart::ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChPushPipeAxle::Output(ChVehicleOutput& database) const {
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

}  // end namespace vehicle
}  // end namespace chrono
