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
// Base class for a DeDion solid axle suspension.
//
// DeDion axles are lightweight solid axles, they don't carry a differential
// gearbox. There are tow guiding joints
//  - longitudinal: spherejoint
//  - lateral: point-to-plane (simplest version),
//             in real life often a Watt linkage is used instead
// Example cars with DeDion axle(s)
//  - Mowag Duro
//  - Smart
//  - Mitsubishi iMiev
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDeDionAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChDeDionAxle::m_pointNames[] = {"SHOCK_A    ", "SHOCK_C    ", "SPRING_A   ", "SPRING_C   ",
                                                  "SPINDLE    ", "AXLE_C     ", "STABI_CON  ", "WATT_CNT_LE",
                                                  "WATT_CNT_RI", "WATT_LE_CH ", "WATT_RI_CH "};

// -----------------------------------------------------------------------------
ChDeDionAxle::ChDeDionAxle(const std::string& name) : ChSuspension(name) {}

ChDeDionAxle::~ChDeDionAxle() {
    if (!IsInitialized())
        return;

    auto sys = m_axleTube->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_axleTube);
    for (int i = 0; i < 2; i++) {
        sys->Remove(m_shock[i]);
        sys->Remove(m_spring[i]);
    }
}

// -----------------------------------------------------------------------------
void ChDeDionAxle::Construct(std::shared_ptr<ChChassis> chassis,
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
    ////ChVector3d midpoint_local = 0.0;
    ////ChVector3d outer_local(axleCOM_local.x(), midpoint_local.y(),
    /// axleCOM_local.z());
    ChVector3d outer_local(getLocation(SPINDLE));
    m_axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    ChVector3d conn_local(getLocation(STABI_CON));
    m_stabiConnectorL = suspension_to_abs.TransformPointLocalToParent(conn_local);
    conn_local.y() = -conn_local.y();
    m_stabiConnectorR = suspension_to_abs.TransformPointLocalToParent(conn_local);

    m_wattLower = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_LE));
    m_wattUpper = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_RI));
    m_wattOuterL = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_LE_CH));
    m_wattOuterR = suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_RI_CH));

    // longitudinal guide, modeled as a pushbar
    m_axleChassis = suspension_to_abs.TransformPointLocalToParent(getLocation(AXLE_C));
    m_axleCenter = (m_axleOuterR + m_axleOuterL) / 2.0;

    // Create and initialize the axle body.
    m_axleTube = chrono_types::make_shared<ChBody>();
    m_axleTube->SetName(m_name + "_axleTube");
    m_axleTube->SetTag(m_obj_tag);
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTube);

    m_axleTubeGuideLong = chrono_types::make_shared<ChLinkLockSpherical>();
    m_axleTubeGuideLong->SetName(m_name + "_sphereAxleTube");
    m_axleTubeGuideLong->SetTag(m_obj_tag);
    ChVector3d spPos = suspension_to_abs.TransformPointLocalToParent(getLocation(AXLE_C));
    m_axleTubeGuideLong->Initialize(m_axleTube, chassis->GetBody(), ChFrame<>(spPos, QUNIT));
    chassis->GetSystem()->AddLink(m_axleTubeGuideLong);

    // Watt lateral guiding mechanism
    ChVector3d cntrPos =
        suspension_to_abs.TransformPointLocalToParent((getLocation(WATT_CNT_LE) + getLocation(WATT_CNT_RI) / 2.0));
    m_wattCenterLinkBody = chrono_types::make_shared<ChBody>();
    m_wattCenterLinkBody->SetName(m_name + "_wattCenterBody");
    m_wattCenterLinkBody->SetTag(m_obj_tag);
    m_wattCenterLinkBody->SetPos(cntrPos);
    m_wattCenterLinkBody->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_wattCenterLinkBody->SetMass(getWattCenterMass());
    m_wattCenterLinkBody->SetInertiaXX(getWattCenterInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_wattCenterLinkBody);

    ChVector3d lftPos =
        suspension_to_abs.TransformPointLocalToParent((getLocation(WATT_LE_CH) + getLocation(WATT_CNT_LE)) / 2.0);
    m_wattLeftLinkBody = chrono_types::make_shared<ChBody>();
    m_wattLeftLinkBody->SetName(m_name + "_wattLeftBody");
    m_wattLeftLinkBody->SetTag(m_obj_tag);
    m_wattLeftLinkBody->SetPos(lftPos);
    m_wattLeftLinkBody->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_wattLeftLinkBody->SetMass(getWattSideMass());
    m_wattLeftLinkBody->SetInertiaXX(getWattSideInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_wattLeftLinkBody);

    ChVector3d rghtPos =
        suspension_to_abs.TransformPointLocalToParent((getLocation(WATT_RI_CH) + getLocation(WATT_CNT_RI)) / 2.0);
    m_wattRightLinkBody = chrono_types::make_shared<ChBody>();
    m_wattRightLinkBody->SetName(m_name + "_wattRightBody");
    m_wattRightLinkBody->SetTag(m_obj_tag);
    m_wattRightLinkBody->SetPos(rghtPos);
    m_wattRightLinkBody->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_wattRightLinkBody->SetMass(getWattSideMass());
    m_wattRightLinkBody->SetInertiaXX(getWattSideInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_wattRightLinkBody);

    // link the Watt center link to the axle tube
    m_wattCenterRev = chrono_types::make_shared<ChLinkLockRevolute>();
    m_wattCenterRev->SetName(m_name + "_wattCenterPivot");
    m_wattCenterRev->SetTag(m_obj_tag);
    m_wattCenterRev->Initialize(m_wattCenterLinkBody, m_axleTube, ChFrame<>(cntrPos, QuatFromAngleY(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_wattCenterRev);

    // link the Watt left link to the center link
    m_wattLeftToCenterSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattLeftToCenterSph->SetName(m_name + "_wattLeft2CenterSph");
    m_wattLeftToCenterSph->SetTag(m_obj_tag);
    m_wattLeftToCenterSph->Initialize(
        m_wattLeftLinkBody, m_wattCenterLinkBody,
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_LE)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattLeftToCenterSph);

    // link the Watt left link to the axle tube
    m_wattLeftToAxleTubeSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattLeftToAxleTubeSph->SetName(m_name + "_wattLeft2ChassisSph");
    m_wattLeftToAxleTubeSph->SetTag(m_obj_tag);
    m_wattLeftToAxleTubeSph->Initialize(
        m_wattLeftLinkBody, chassis->GetBody(),
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_LE_CH)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattLeftToAxleTubeSph);

    // link the Watt right link to the center link
    m_wattRightToCenterSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattRightToCenterSph->SetName(m_name + "_wattRight2CenterSph");
    m_wattRightToCenterSph->SetTag(m_obj_tag);
    m_wattRightToCenterSph->Initialize(
        m_wattRightLinkBody, m_wattCenterLinkBody,
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_CNT_RI)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattRightToCenterSph);

    // link the Watt right link to the axle tube
    m_wattRightToAxleTubeSph = chrono_types::make_shared<ChLinkLockSpherical>();
    m_wattRightToAxleTubeSph->SetName(m_name + "_wattRight2ChassisSph");
    m_wattRightToAxleTubeSph->SetTag(m_obj_tag);
    m_wattRightToAxleTubeSph->Initialize(
        m_wattRightLinkBody, chassis->GetBody(),
        ChFrame<>(suspension_to_abs.TransformPointLocalToParent(getLocation(WATT_RI_CH)), QUNIT));
    chassis->GetSystem()->AddLink(m_wattRightToAxleTubeSph);

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
    std::shared_ptr<ChBody> scbeamL = (subchassis == nullptr) ? chassis->GetBody() : subchassis->GetBeam(LEFT);
    std::shared_ptr<ChBody> scbeamR = (subchassis == nullptr) ? chassis->GetBody() : subchassis->GetBeam(RIGHT);
    InitializeSide(LEFT, chassis->GetBody(), scbeamL, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), scbeamR, m_pointsR, right_ang_vel);
}

void ChDeDionAxle::InitializeSide(VehicleSide side,
                                  std::shared_ptr<ChBodyAuxRef> chassis,
                                  std::shared_ptr<ChBody> scbeam,
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

    // Initialize spindle body (same orientation as the chassis)
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());

    // Create and initialize the revolute joint between axle tube and spindle.
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->SetTag(m_obj_tag);
    m_revolute[side]->Initialize(m_spindle[side], m_axleTube,
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the shock damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->SetTag(m_obj_tag);
    m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    // Create and initialize the spring
    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetName(m_name + "_spring" + suffix);
    m_spring[side]->SetTag(m_obj_tag);
    m_spring[side]->Initialize(scbeam, m_axleTube, false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle.
    // Note that the spindle rotates about the Y axis.
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

const ChVector3d ChDeDionAxle::GetConnectorLocation(VehicleSide side) {
    if (side == RIGHT) {
        return m_stabiConnectorR;
    } else {
        return m_stabiConnectorL;
    }
}

void ChDeDionAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + 2 * (getSpindleMass() + getWattCenterMass() + 2 * getWattSideMass());
}

void ChDeDionAxle::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaWattCenter(getWattCenterInertia());
    ChMatrix33<> inertiaWattSide(getWattSideInertia());

    CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_axleTube->GetFrameCOMToAbs(), getAxleTubeMass(), ChMatrix33<>(getAxleInertia()));
    composite.AddComponent(m_wattCenterLinkBody->GetFrameCOMToAbs(), getWattCenterMass(),
                           ChMatrix33<>(inertiaWattCenter));
    composite.AddComponent(m_wattLeftLinkBody->GetFrameCOMToAbs(), getWattSideMass(), ChMatrix33<>(inertiaWattSide));
    composite.AddComponent(m_wattRightLinkBody->GetFrameCOMToAbs(), getWattSideMass(), ChMatrix33<>(inertiaWattSide));

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChDeDionAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChDeDionAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDeDionAxle::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDeDionAxle::LogConstraintViolations(VehicleSide side) {
    {
        /*
      ChVectorDynamic<> C = m_axleTubeGuideLat->GetConstraintViolation();
      std::cout << "Axle tube prismatic       ";
      std::cout << "  " << C(0) << "  ";
      std::cout << "  " << C(1) << "  ";
      std::cout << "  " << C(2) << "\n";
         */
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDeDionAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_axleTube, m_axleCenter, m_axleChassis, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_axleTube, m_stabiConnectorL, m_stabiConnectorR, getAxleTubeRadius() / 2,
                         ChColor(0.7f, 0.7f, 0.7f));
    AddVisualizationLink(m_wattCenterLinkBody, m_wattLower, m_wattUpper, getWattLinkRadius(),
                         ChColor(0.5f, 0.7f, 0.8f));
    AddVisualizationLink(m_wattLeftLinkBody, m_wattLower, m_wattOuterL, getWattLinkRadius(), ChColor(0.8f, 0.5f, 0.5f));
    AddVisualizationLink(m_wattRightLinkBody, m_wattUpper, m_wattOuterR, getWattLinkRadius(),
                         ChColor(0.5f, 0.8f, 0.5f));

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChDeDionAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);
    ChPart::RemoveVisualizationAssets(m_wattCenterLinkBody);
    ChPart::RemoveVisualizationAssets(m_wattLeftLinkBody);
    ChPart::RemoveVisualizationAssets(m_wattRightLinkBody);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDeDionAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                        const ChVector3d pt_1,
                                        const ChVector3d pt_2,
                                        double radius,
                                        const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector3d p_2 = body->TransformPointParentToLocal(pt_2);

    auto cyl = utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
    cyl->SetColor(color);
}

// -----------------------------------------------------------------------------

void ChDeDionAxle::PopulateComponentList() {
    m_bodies.push_back(m_spindle[0]);
    m_bodies.push_back(m_spindle[1]);
    m_bodies.push_back(m_axleTube);

    m_shafts.push_back(m_axle[0]);
    m_shafts.push_back(m_axle[1]);

    m_joints.push_back(m_revolute[0]);
    m_joints.push_back(m_revolute[1]);

    m_tsdas.push_back(m_spring[0]);
    m_tsdas.push_back(m_spring[1]);
    m_tsdas.push_back(m_shock[0]);
    m_tsdas.push_back(m_shock[1]);
}

}  // end namespace vehicle
}  // end namespace chrono
