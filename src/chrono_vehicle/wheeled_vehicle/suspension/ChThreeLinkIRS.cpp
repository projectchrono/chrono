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
// Authors: Radu Serban
// =============================================================================
//
// Base class for a 3-link independent rear suspension (non-steerable).
// This suspension has a trailing arm and 2 additional links connecting the
// arm to the chassis.
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

#include <algorithm>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChThreeLinkIRS.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChThreeLinkIRS::m_pointNames[] = {"SPINDLE ", "TA_CM",    "TA_O",     "TA_I",    "TA_S",
                                                    "SHOCK_C ", "SHOCK_A ", "SPRING_C", "SPRING_A"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChThreeLinkIRS::ChThreeLinkIRS(const std::string& name) : ChSuspension(name) {}

ChThreeLinkIRS::~ChThreeLinkIRS() {
    if (!m_initialized)
        return;

    auto sys = m_arm[0]->GetSystem();
    if (!sys)
        return;

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_arm[i]);
        sys->Remove(m_upper[i]);
        sys->Remove(m_lower[i]);
        ChChassis::RemoveJoint(m_sphericalArm[i]);
        ChChassis::RemoveJoint(m_sphericalUpper[i]);
        ChChassis::RemoveJoint(m_sphericalLower[i]);
        ChChassis::RemoveJoint(m_universalUpper[i]);
        ChChassis::RemoveJoint(m_universalLower[i]);
        sys->Remove(m_shock[i]);
        sys->Remove(m_spring[i]);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::Initialize(std::shared_ptr<ChChassis> chassis,
                                std::shared_ptr<ChSubchassis> subchassis,
                                std::shared_ptr<ChSteering> steering,
                                const ChVector3d& location,
                                double left_ang_vel,
                                double right_ang_vel) {
    ChSuspension::Initialize(chassis, subchassis, steering, location, left_ang_vel, right_ang_vel);

    m_parent = chassis;
    m_rel_loc = location;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform all hardpoints and directions to absolute frame.
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);

    m_dirsL.resize(NUM_DIRS);
    m_dirsR.resize(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformPointLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector3d rel_dir = getDirection(static_cast<DirectionId>(i));
        m_dirsL[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
        rel_dir.y() = -rel_dir.y();
        m_dirsR[i] = suspension_to_abs.TransformDirectionLocalToParent(rel_dir);
    }

    // Initialize left and right sides.
    InitializeSide(LEFT, chassis, m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, m_pointsR, m_dirsR, right_ang_vel);
}

void ChThreeLinkIRS::InitializeSide(VehicleSide side,
                                    std::shared_ptr<ChChassis> chassis,
                                    const std::vector<ChVector3d>& points,
                                    const std::vector<ChVector3d>& dirs,
                                    double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrameRefToAbs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * QuatFromAngleZ(sign * getToeAngle()) * QuatFromAngleX(sign * getCamberAngle());

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = chrono_types::make_shared<ChBody>();
    m_spindle[side]->SetName(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Create and initialize the trailing arm and the two link bodies.
    u = points[TA_C] - points[TA_S];
    u.Normalize();
    v = Vcross(ChVector3d(0, 0, 1), u);
    v.Normalize();
    w = Vcross(u, v);
    rot.SetFromDirectionAxes(u, v, w);

    m_arm[side] = chrono_types::make_shared<ChBody>();
    m_arm[side]->SetName(m_name + "_arm" + suffix);
    m_arm[side]->SetPos(points[TA_CM]);
    m_arm[side]->SetRot(rot);
    m_arm[side]->SetMass(getArmMass());
    m_arm[side]->SetInertiaXX(getArmInertia());
    chassis->GetSystem()->AddBody(m_arm[side]);

    u = points[UL_A] - points[UL_C];
    u.Normalize();
    v = Vcross(ChVector3d(0, 0, 1), u);
    v.Normalize();
    w = Vcross(u, v);
    rot.SetFromDirectionAxes(u, v, w);

    m_upper[side] = chrono_types::make_shared<ChBody>();
    m_upper[side]->SetName(m_name + "_upper" + suffix);
    m_upper[side]->SetPos(points[UL_CM]);
    m_upper[side]->SetRot(rot);
    m_upper[side]->SetMass(getUpperLinkMass());
    m_upper[side]->SetInertiaXX(getUpperLinkInertia());
    chassis->GetSystem()->AddBody(m_upper[side]);

    u = points[LL_A] - points[LL_C];
    u.Normalize();
    v = Vcross(ChVector3d(0, 0, 1), u);
    v.Normalize();
    w = Vcross(u, v);
    rot.SetFromDirectionAxes(u, v, w);

    m_lower[side] = chrono_types::make_shared<ChBody>();
    m_lower[side]->SetName(m_name + "_lower" + suffix);
    m_lower[side]->SetPos(points[LL_CM]);
    m_lower[side]->SetRot(rot);
    m_lower[side]->SetMass(getLowerLinkMass());
    m_lower[side]->SetInertiaXX(getLowerLinkInertia());
    chassis->GetSystem()->AddBody(m_lower[side]);

    // Create and initialize the revolute joint between arm and spindle.
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_arm[side],
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spherical joint between chassis and arm.
    m_sphericalArm[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalArm" + suffix, chassis->GetBody(), m_arm[side],
        ChFrame<>(points[TA_C], QUNIT), getArmChassisBushingData());
    chassis->AddJoint(m_sphericalArm[side]);

    // Create and initialize the spherical joints between links and arm.
    m_sphericalUpper[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalUpper" + suffix, m_upper[side], m_arm[side],
        ChFrame<>(points[UL_A], QUNIT), getArmUpperBushingData());
    chassis->AddJoint(m_sphericalUpper[side]);

    m_sphericalLower[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalLower" + suffix, m_lower[side], m_arm[side],
        ChFrame<>(points[LL_A], QUNIT), getArmLowerBushingData());
    chassis->AddJoint(m_sphericalLower[side]);

    // Create and initialize the universal joints between links and chassis.
    u = dirs[UNIV_AXIS_UPPER];
    w = Vcross(u, ChVector3d(0, 0, 1));
    w.Normalize();
    v = Vcross(w, u);
    rot.SetFromDirectionAxes(u, v, w);

    m_universalUpper[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::UNIVERSAL, m_name + "_universalUpper" + suffix, m_upper[side], chassis->GetBody(),
        ChFrame<>(points[UL_C], rot.GetQuaternion()), getChassisUpperBushingData());
    chassis->AddJoint(m_universalUpper[side]);

    u = dirs[UNIV_AXIS_LOWER];
    w = Vcross(u, ChVector3d(0, 0, 1));
    w.Normalize();
    v = Vcross(w, u);
    rot.SetFromDirectionAxes(u, v, w);

    m_universalLower[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::UNIVERSAL, m_name + "_universalLower" + suffix, m_lower[side], chassis->GetBody(),
        ChFrame<>(points[LL_C], rot.GetQuaternion()), getChassisLowerBushingData());
    chassis->AddJoint(m_universalLower[side]);

    // Create and initialize the spring/damper.
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis->GetBody(), m_arm[side], false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetName(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis->GetBody(), m_arm[side], false, points[SPRING_C], points[SPRING_A]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

    // Create and initialize the axle shaft and its connection to the spindle. Note that the
    // spindle rotates about the Y axis.
    m_axle[side] = chrono_types::make_shared<ChShaft>();
    m_axle[side]->SetName(m_name + "_axle" + suffix);
    m_axle[side]->SetInertia(getAxleInertia());
    m_axle[side]->SetPosDt(-ang_vel);
    chassis->GetSystem()->AddShaft(m_axle[side]);

    m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftBodyRotation>();
    m_axle_to_spindle[side]->SetName(m_name + "_axle_to_spindle" + suffix);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector3d(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

void ChThreeLinkIRS::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getArmMass() + getLowerLinkMass() + getUpperLinkMass());
}

void ChThreeLinkIRS::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaArm(getArmInertia());
    ChMatrix33<> inertiaLower(getLowerLinkInertia());
    ChMatrix33<> inertiaUpper(getUpperLinkInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_arm[LEFT]->GetFrameCOMToAbs(), getArmMass(), inertiaArm);
    composite.AddComponent(m_arm[RIGHT]->GetFrameCOMToAbs(), getArmMass(), inertiaArm);
    composite.AddComponent(m_lower[LEFT]->GetFrameCOMToAbs(), getLowerLinkMass(), inertiaLower);
    composite.AddComponent(m_lower[RIGHT]->GetFrameCOMToAbs(), getLowerLinkMass(), inertiaLower);
    composite.AddComponent(m_upper[LEFT]->GetFrameCOMToAbs(), getUpperLinkMass(), inertiaUpper);
    composite.AddComponent(m_upper[RIGHT]->GetFrameCOMToAbs(), getUpperLinkMass(), inertiaUpper);

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChThreeLinkIRS::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChThreeLinkIRS::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::LogConstraintViolations(VehicleSide side) {
    {
        ChVectorDynamic<> C = m_sphericalArm[side]->GetConstraintViolation();
        std::cout << "Arm spherical         ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalUpper[side]->GetConstraintViolation();
        std::cout << "Upper spherical       ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalLower[side]->GetConstraintViolation();
        std::cout << "Lower spherical       ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalUpper[side]->GetConstraintViolation();
        std::cout << "Upper universal       ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalLower[side]->GetConstraintViolation();
        std::cout << "Lower universal       ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revolute[side]->GetConstraintViolation();
        std::cout << "Spindle revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for trailing arms
    AddVisualizationArm(m_arm[LEFT], m_pointsL[TA_C], m_pointsL[TA_S], m_pointsL[TA_CM], m_pointsL[UL_A],
                        m_pointsL[LL_A], getArmRadius());
    AddVisualizationArm(m_arm[RIGHT], m_pointsR[TA_C], m_pointsR[TA_S], m_pointsR[TA_CM], m_pointsR[UL_A],
                        m_pointsR[LL_A], getArmRadius());

    // Add visualization for upper links
    AddVisualizationLink(m_upper[LEFT], m_pointsL[UL_C], m_pointsL[UL_A], m_pointsL[UL_CM], getUpperLinkRadius());
    AddVisualizationLink(m_upper[RIGHT], m_pointsR[UL_C], m_pointsR[UL_A], m_pointsR[UL_CM], getUpperLinkRadius());

    // Add visualization for lower links
    AddVisualizationLink(m_lower[LEFT], m_pointsL[LL_C], m_pointsL[LL_A], m_pointsL[LL_CM], getLowerLinkRadius());
    AddVisualizationLink(m_lower[RIGHT], m_pointsR[LL_C], m_pointsR[LL_A], m_pointsR[LL_CM], getLowerLinkRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChThreeLinkIRS::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_arm[LEFT]);
    ChPart::RemoveVisualizationAssets(m_arm[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_upper[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upper[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_lower[LEFT]);
    ChPart::RemoveVisualizationAssets(m_lower[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::AddVisualizationArm(std::shared_ptr<ChBody> body,
                                         const ChVector3d& pt_C,
                                         const ChVector3d& pt_S,
                                         const ChVector3d& pt_CM,
                                         const ChVector3d& pt_U,
                                         const ChVector3d& pt_L,
                                         double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector3d p_C = body->TransformPointParentToLocal(pt_C);
    ChVector3d p_S = body->TransformPointParentToLocal(pt_S);
    ChVector3d p_CM = body->TransformPointParentToLocal(pt_CM);
    ChVector3d p_U = body->TransformPointParentToLocal(pt_U);
    ChVector3d p_L = body->TransformPointParentToLocal(pt_L);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_C, p_CM, radius);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_S, p_CM, radius);

    if ((p_S - p_U).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(body, p_S, p_U, radius);
    }

    if ((p_S - p_L).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(body, p_S, p_L, radius);
    }
}

void ChThreeLinkIRS::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                          const ChVector3d& pt_1,
                                          const ChVector3d& pt_2,
                                          const ChVector3d& pt_CM,
                                          double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector3d p_2 = body->TransformPointParentToLocal(pt_2);
    ChVector3d p_CM = body->TransformPointParentToLocal(pt_CM);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_CM, radius);
    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_2, p_CM, radius);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChThreeLinkIRS::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    bodies.push_back(m_upper[0]);
    bodies.push_back(m_upper[1]);
    bodies.push_back(m_lower[0]);
    bodies.push_back(m_lower[1]);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    m_sphericalArm[0]->IsKinematic() ? joints.push_back(m_sphericalArm[0]->GetAsLink())
                                     : bushings.push_back(m_sphericalArm[0]->GetAsBushing());
    m_sphericalArm[1]->IsKinematic() ? joints.push_back(m_sphericalArm[1]->GetAsLink())
                                     : bushings.push_back(m_sphericalArm[1]->GetAsBushing());
    m_sphericalUpper[0]->IsKinematic() ? joints.push_back(m_sphericalUpper[0]->GetAsLink())
                                       : bushings.push_back(m_sphericalUpper[0]->GetAsBushing());
    m_sphericalUpper[1]->IsKinematic() ? joints.push_back(m_sphericalUpper[1]->GetAsLink())
                                       : bushings.push_back(m_sphericalUpper[1]->GetAsBushing());
    m_sphericalLower[0]->IsKinematic() ? joints.push_back(m_sphericalLower[0]->GetAsLink())
                                       : bushings.push_back(m_sphericalLower[0]->GetAsBushing());
    m_sphericalLower[1]->IsKinematic() ? joints.push_back(m_sphericalLower[1]->GetAsLink())
                                       : bushings.push_back(m_sphericalLower[1]->GetAsBushing());
    m_universalUpper[0]->IsKinematic() ? joints.push_back(m_universalUpper[0]->GetAsLink())
                                       : bushings.push_back(m_universalUpper[0]->GetAsBushing());
    m_universalUpper[1]->IsKinematic() ? joints.push_back(m_universalUpper[1]->GetAsLink())
                                       : bushings.push_back(m_universalUpper[1]->GetAsBushing());
    m_universalLower[0]->IsKinematic() ? joints.push_back(m_universalLower[0]->GetAsLink())
                                       : bushings.push_back(m_universalLower[0]->GetAsBushing());
    m_universalLower[1]->IsKinematic() ? joints.push_back(m_universalLower[1]->GetAsLink())
                                       : bushings.push_back(m_universalLower[1]->GetAsBushing());
    ExportJointList(jsonDocument, joints);
    ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChThreeLinkIRS::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_arm[0]);
    bodies.push_back(m_arm[1]);
    bodies.push_back(m_upper[0]);
    bodies.push_back(m_upper[1]);
    bodies.push_back(m_lower[0]);
    bodies.push_back(m_lower[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    m_sphericalArm[0]->IsKinematic() ? joints.push_back(m_sphericalArm[0]->GetAsLink())
                                     : bushings.push_back(m_sphericalArm[0]->GetAsBushing());
    m_sphericalArm[1]->IsKinematic() ? joints.push_back(m_sphericalArm[1]->GetAsLink())
                                     : bushings.push_back(m_sphericalArm[1]->GetAsBushing());
    m_sphericalUpper[0]->IsKinematic() ? joints.push_back(m_sphericalUpper[0]->GetAsLink())
                                       : bushings.push_back(m_sphericalUpper[0]->GetAsBushing());
    m_sphericalUpper[1]->IsKinematic() ? joints.push_back(m_sphericalUpper[1]->GetAsLink())
                                       : bushings.push_back(m_sphericalUpper[1]->GetAsBushing());
    m_sphericalLower[0]->IsKinematic() ? joints.push_back(m_sphericalLower[0]->GetAsLink())
                                       : bushings.push_back(m_sphericalLower[0]->GetAsBushing());
    m_sphericalLower[1]->IsKinematic() ? joints.push_back(m_sphericalLower[1]->GetAsLink())
                                       : bushings.push_back(m_sphericalLower[1]->GetAsBushing());
    m_universalUpper[0]->IsKinematic() ? joints.push_back(m_universalUpper[0]->GetAsLink())
                                       : bushings.push_back(m_universalUpper[0]->GetAsBushing());
    m_universalUpper[1]->IsKinematic() ? joints.push_back(m_universalUpper[1]->GetAsLink())
                                       : bushings.push_back(m_universalUpper[1]->GetAsBushing());
    m_universalLower[0]->IsKinematic() ? joints.push_back(m_universalLower[0]->GetAsLink())
                                       : bushings.push_back(m_universalLower[0]->GetAsBushing());
    m_universalLower[1]->IsKinematic() ? joints.push_back(m_universalLower[1]->GetAsLink())
                                       : bushings.push_back(m_universalLower[1]->GetAsBushing());
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
