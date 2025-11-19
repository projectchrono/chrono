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
// Base class for a single-A arm suspension modeled with bodies and constraints.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChSingleWishbone.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChSingleWishbone::m_pointNames[] = {"SPINDLE ", "UPRIGHT ", "CA_C    ", "CA_U    ", "CA_CM   ",
                                                      "STRUT_C ", "STRUT_A ", "TIEROD_C", "TIEROD_U"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChSingleWishbone::ChSingleWishbone(const std::string& name) : ChSuspension(name) {}

ChSingleWishbone::~ChSingleWishbone() {
    if (!m_initialized)
        return;

    auto sys = m_upright[0]->GetSystem();
    if (!sys)
        return;

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_upright[i]);
        sys->Remove(m_control_arm[i]);

        ChChassis::RemoveJoint(m_revoluteUA[i]);
        ChChassis::RemoveJoint(m_revoluteUA[i]);

        if (m_tierod[i]) {
            sys->Remove(m_tierod[i]);
            ChChassis::RemoveJoint(m_sphericalTierod[i]);
            ChChassis::RemoveJoint(m_universalTierod[i]);
        }
        if (m_distTierod[i]) {
            sys->Remove(m_distTierod[i]);
        }

        sys->Remove(m_shock[i]);
    }
}

// -----------------------------------------------------------------------------
void ChSingleWishbone::Construct(std::shared_ptr<ChChassis> chassis,
                                 std::shared_ptr<ChSubchassis> subchassis,
                                 std::shared_ptr<ChSteering> steering,
                                 const ChVector3d& location,
                                 double left_ang_vel,
                                 double right_ang_vel) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

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
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, right_ang_vel);
}

void ChSingleWishbone::InitializeSide(VehicleSide side,
                                      std::shared_ptr<ChChassis> chassis,
                                      std::shared_ptr<ChBody> tierod_body,
                                      const std::vector<ChVector3d>& points,
                                      double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrameRefToAbs().GetRot();
    ChVector3d up = chassisRot.GetAxisZ();

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * QuatFromAngleZ(sign * getToeAngle()) * QuatFromAngleX(sign * getCamberAngle());

    // Initialize the spindle body
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());

    // Create and initialize the upright body
    m_upright[side] = chrono_types::make_shared<ChBody>();
    m_upright[side]->SetName(m_name + "_upright" + suffix);
    m_upright[side]->SetTag(m_obj_tag);
    m_upright[side]->SetPos(points[UPRIGHT]);
    m_upright[side]->SetRot(chassisRot);
    m_upright[side]->SetMass(getUprightMass());
    m_upright[side]->SetInertiaXX(getUprightInertiaMoments());
    m_upright[side]->SetInertiaXY(getUprightInertiaProducts());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Orientation of the control arm.
    // Y axis along the direction of the control arm (connections to chassis and to upright).
    v = (points[CA_U] - points[CA_C]).GetNormalized();
    u = Vcross(v, up);
    w = Vcross(u, v);
    rot.SetFromDirectionAxes(u, v, w);

    // Create and initialize the control arm body
    m_control_arm[side] = chrono_types::make_shared<ChBody>();
    m_control_arm[side]->SetName(m_name + "_arm" + suffix);
    m_control_arm[side]->SetTag(m_obj_tag);
    m_control_arm[side]->SetPos(points[CA_CM]);
    m_control_arm[side]->SetRot(rot.GetQuaternion());
    m_control_arm[side]->SetMass(getCAMass());
    m_control_arm[side]->SetInertiaXX(getCAInertiaMoments());
    m_control_arm[side]->SetInertiaXY(getCAInertiaProducts());
    chassis->GetSystem()->AddBody(m_control_arm[side]);

    // Create and initialize the revolute joint between upright and spindle
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->SetTag(m_obj_tag);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side],
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the revolute joint between chassis and CA
    m_revoluteCA[side] = chrono_types::make_shared<ChJoint>(
        ChJoint::Type::REVOLUTE, m_name + "_revoluteCA" + suffix, chassis->GetBody(), m_control_arm[side],
        ChFrame<>(points[CA_C], chassisRot * QuatFromAngleY(CH_PI_2)), getCABushingData());
    m_revoluteCA[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_revoluteCA[side]);

    // Create and initialize the revolute joint between upright and CA
    m_revoluteUA[side] = chrono_types::make_shared<ChJoint>(
        ChJoint::Type::REVOLUTE, m_name + "_revoluteUA" + suffix, m_control_arm[side], m_upright[side],
        ChFrame<>(points[CA_U], chassisRot));
    m_revoluteUA[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_revoluteUA[side]);

    if (UseTierodBodies()) {
        // Orientation of tierod body
        w = (points[TIEROD_U] - points[TIEROD_C]).GetNormalized();
        u = chassisRot.GetAxisX();
        v = Vcross(w, u).GetNormalized();
        u = Vcross(v, w);
        rot.SetFromDirectionAxes(u, v, w);

        // Create the tierod body
        m_tierod[side] = chrono_types::make_shared<ChBody>();
        m_tierod[side]->SetName(m_name + "_tierodBody" + suffix);
        m_tierod[side]->SetTag(m_obj_tag);
        m_tierod[side]->SetPos((points[TIEROD_U] + points[TIEROD_C]) / 2);
        m_tierod[side]->SetRot(rot.GetQuaternion());
        m_tierod[side]->SetMass(getTierodMass());
        m_tierod[side]->SetInertiaXX(getTierodInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_tierod[side]);

        // Connect tierod body to upright (spherical) and chassis (universal)
        m_sphericalTierod[side] = chrono_types::make_shared<ChJoint>(
            ChJoint::Type::SPHERICAL, m_name + "_sphericalTierod" + suffix, m_upright[side], m_tierod[side],
            ChFrame<>(points[TIEROD_U], QUNIT), getTierodBushingData());
        m_sphericalTierod[side]->SetTag(m_obj_tag);
        chassis->AddJoint(m_sphericalTierod[side]);
        m_universalTierod[side] = chrono_types::make_shared<ChJoint>(
            ChJoint::Type::UNIVERSAL, m_name + "_universalTierod" + suffix, tierod_body, m_tierod[side],
            ChFrame<>(points[TIEROD_C], rot.GetQuaternion()), getTierodBushingData());
        m_universalTierod[side]->SetTag(m_obj_tag);
        chassis->AddJoint(m_universalTierod[side]);
    } else {
        // Create and initialize the tierod distance constraint between chassis and upright.
        m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
        m_distTierod[side]->SetName(m_name + "_distTierod" + suffix);
        m_distTierod[side]->SetTag(m_obj_tag);
        m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
        chassis->GetSystem()->AddLink(m_distTierod[side]);
    }

    // Create and initialize the spring-damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->SetTag(m_obj_tag);
    m_shock[side]->Initialize(chassis->GetBody(), m_control_arm[side], false, points[STRUT_C], points[STRUT_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

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
    m_axle_to_spindle[side]->SetTag(m_obj_tag);
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector3d(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

void ChSingleWishbone::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getCAMass() + getUprightMass());
    if (UseTierodBodies()) {
        m_mass += 2 * getTierodMass();
    }
}

void ChSingleWishbone::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaCA(getCAInertiaMoments(), getCAInertiaProducts());
    ChMatrix33<> inertiaUpright(getUprightInertiaMoments(), getUprightInertiaProducts());

    CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_control_arm[LEFT]->GetFrameCOMToAbs(), getCAMass(), inertiaCA);
    composite.AddComponent(m_control_arm[RIGHT]->GetFrameCOMToAbs(), getCAMass(), inertiaCA);
    composite.AddComponent(m_upright[LEFT]->GetFrameCOMToAbs(), getUprightMass(), inertiaUpright);
    composite.AddComponent(m_upright[RIGHT]->GetFrameCOMToAbs(), getUprightMass(), inertiaUpright);

    if (UseTierodBodies()) {
        ChMatrix33<> inertiaTierod(getTierodInertia());
        composite.AddComponent(m_tierod[LEFT]->GetFrameCOMToAbs(), getTierodMass(), inertiaTierod);
        composite.AddComponent(m_tierod[RIGHT]->GetFrameCOMToAbs(), getTierodMass(), inertiaTierod);
    }

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChSingleWishbone::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChSingleWishbone::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(1);

    forces[0] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteCA[side]->GetConstraintViolation();
        std::cout << "LCA revolute          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revoluteUA[side]->GetConstraintViolation();
        std::cout << "UCA revolute          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
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

    // Tierod constraint
    if (UseTierodBodies()) {
        {
            const auto& C = m_sphericalTierod[side]->GetConstraintViolation();
            std::cout << "Tierod spherical      ";
            std::cout << "  " << C(0) << "  ";
            std::cout << "  " << C(1) << "  ";
            std::cout << "  " << C(2) << "\n";
        }
        {
            const auto& C = m_universalTierod[side]->GetConstraintViolation();
            std::cout << "Tierod universal      ";
            std::cout << "  " << C(0) << "  ";
            std::cout << "  " << C(1) << "  ";
            std::cout << "  " << C(2) << "\n";
            std::cout << "  " << C(3) << "\n";
        }
    } else {
        std::cout << "Tierod distance       ";
        std::cout << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance()
                  << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for uprights
    AddVisualizationUpright(m_upright[LEFT], m_pointsL[UPRIGHT], m_pointsL[SPINDLE], m_pointsL[CA_U],
                            m_pointsL[TIEROD_U], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], m_pointsR[UPRIGHT], m_pointsR[SPINDLE], m_pointsR[CA_U],
                            m_pointsR[TIEROD_U], getUprightRadius());

    // Add visualization for lower control arms
    AddVisualizationControlArm(m_control_arm[LEFT], m_pointsL[CA_C], m_pointsL[CA_U], getCARadius());
    AddVisualizationControlArm(m_control_arm[RIGHT], m_pointsR[CA_C], m_pointsR[CA_U], getCARadius());

    // Add visualization for the shocks
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.04, 150, 15));
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.04, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    // Add visualization for the tie-rods
    if (UseTierodBodies()) {
        AddVisualizationTierod(m_tierod[LEFT], m_pointsL[TIEROD_C], m_pointsL[TIEROD_U], getTierodRadius());
        AddVisualizationTierod(m_tierod[RIGHT], m_pointsR[TIEROD_C], m_pointsR[TIEROD_U], getTierodRadius());
    } else {
        m_distTierod[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
        m_distTierod[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    }
}

void ChSingleWishbone::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_upright[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upright[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_control_arm[LEFT]);
    ChPart::RemoveVisualizationAssets(m_control_arm[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    if (UseTierodBodies()) {
        ChPart::RemoveVisualizationAssets(m_tierod[LEFT]);
        ChPart::RemoveVisualizationAssets(m_tierod[RIGHT]);
    } else {
        ChPart::RemoveVisualizationAssets(m_distTierod[LEFT]);
        ChPart::RemoveVisualizationAssets(m_distTierod[RIGHT]);
    }

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::AddVisualizationControlArm(std::shared_ptr<ChBody> arm,
                                                  const ChVector3d pt_C,
                                                  const ChVector3d pt_U,
                                                  double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_C = arm->TransformPointParentToLocal(pt_C);
    ChVector3d p_U = arm->TransformPointParentToLocal(pt_U);

    utils::ChBodyGeometry::AddVisualizationCylinder(arm, p_C, p_U, radius);
    utils::ChBodyGeometry::AddVisualizationCylinder(arm, p_C + ChVector3d(radius, 0, 0), p_C - ChVector3d(radius, 0, 0),
                                                    radius);
}

void ChSingleWishbone::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                               const ChVector3d pt_U,
                                               const ChVector3d pt_S,
                                               const ChVector3d pt_A,
                                               const ChVector3d pt_T,
                                               double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector3d p_U = upright->TransformPointParentToLocal(pt_U);  // upright center
    ChVector3d p_S = upright->TransformPointParentToLocal(pt_S);  // spindle center
    ChVector3d p_A = upright->TransformPointParentToLocal(pt_A);  // connection to arm
    ChVector3d p_T = upright->TransformPointParentToLocal(pt_T);  // connection to tierod

    if ((p_U - p_S).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(upright, p_U, p_S, radius);
    }

    if ((p_U - p_A).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(upright, p_U, p_A, radius);
    }

    if ((p_U - p_T).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(upright, p_U, p_T, radius);
    }
}

void ChSingleWishbone::AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
                                              const ChVector3d pt_C,
                                              const ChVector3d pt_U,
                                              double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_C = tierod->TransformPointParentToLocal(pt_C);
    ChVector3d p_U = tierod->TransformPointParentToLocal(pt_U);

    utils::ChBodyGeometry::AddVisualizationCylinder(tierod, p_C, p_U, radius);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChSingleWishbone::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_control_arm[0]);
    bodies.push_back(m_control_arm[1]);
    if (UseTierodBodies()) {
        bodies.push_back(m_tierod[0]);
        bodies.push_back(m_tierod[1]);
    }
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    m_revoluteCA[0]->IsKinematic() ? joints.push_back(m_revoluteCA[0]->GetAsLink())
                                   : bushings.push_back(m_revoluteCA[0]->GetAsBushing());
    m_revoluteCA[1]->IsKinematic() ? joints.push_back(m_revoluteCA[1]->GetAsLink())
                                   : bushings.push_back(m_revoluteCA[1]->GetAsBushing());
    m_revoluteUA[0]->IsKinematic() ? joints.push_back(m_revoluteUA[0]->GetAsLink())
                                   : bushings.push_back(m_revoluteUA[0]->GetAsBushing());
    m_revoluteUA[1]->IsKinematic() ? joints.push_back(m_revoluteUA[1]->GetAsLink())
                                   : bushings.push_back(m_revoluteUA[1]->GetAsBushing());
    if (UseTierodBodies()) {
        m_sphericalTierod[0]->IsKinematic() ? joints.push_back(m_sphericalTierod[0]->GetAsLink())
                                            : bushings.push_back(m_sphericalTierod[0]->GetAsBushing());
        m_sphericalTierod[1]->IsKinematic() ? joints.push_back(m_sphericalTierod[1]->GetAsLink())
                                            : bushings.push_back(m_sphericalTierod[1]->GetAsBushing());
        m_universalTierod[0]->IsKinematic() ? joints.push_back(m_universalTierod[0]->GetAsLink())
                                            : bushings.push_back(m_universalTierod[0]->GetAsBushing());
        m_universalTierod[1]->IsKinematic() ? joints.push_back(m_universalTierod[1]->GetAsLink())
                                            : bushings.push_back(m_universalTierod[1]->GetAsBushing());
    } else {
        joints.push_back(m_distTierod[0]);
        joints.push_back(m_distTierod[1]);
    }
    ExportJointList(jsonDocument, joints);
    ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChSingleWishbone::Output(ChOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_control_arm[0]);
    bodies.push_back(m_control_arm[1]);
    if (UseTierodBodies()) {
        bodies.push_back(m_tierod[0]);
        bodies.push_back(m_tierod[1]);
    }
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    m_revoluteCA[0]->IsKinematic() ? joints.push_back(m_revoluteCA[0]->GetAsLink())
                                   : bushings.push_back(m_revoluteCA[0]->GetAsBushing());
    m_revoluteCA[1]->IsKinematic() ? joints.push_back(m_revoluteCA[1]->GetAsLink())
                                   : bushings.push_back(m_revoluteCA[1]->GetAsBushing());
    m_revoluteUA[0]->IsKinematic() ? joints.push_back(m_revoluteUA[0]->GetAsLink())
                                   : bushings.push_back(m_revoluteUA[0]->GetAsBushing());
    m_revoluteUA[1]->IsKinematic() ? joints.push_back(m_revoluteUA[1]->GetAsLink())
                                   : bushings.push_back(m_revoluteUA[1]->GetAsBushing());
    if (UseTierodBodies()) {
        m_sphericalTierod[0]->IsKinematic() ? joints.push_back(m_sphericalTierod[0]->GetAsLink())
                                            : bushings.push_back(m_sphericalTierod[0]->GetAsBushing());
        m_sphericalTierod[1]->IsKinematic() ? joints.push_back(m_sphericalTierod[1]->GetAsLink())
                                            : bushings.push_back(m_sphericalTierod[1]->GetAsBushing());
        m_universalTierod[0]->IsKinematic() ? joints.push_back(m_universalTierod[0]->GetAsLink())
                                            : bushings.push_back(m_universalTierod[0]->GetAsBushing());
        m_universalTierod[1]->IsKinematic() ? joints.push_back(m_universalTierod[1]->GetAsLink())
                                            : bushings.push_back(m_universalTierod[1]->GetAsBushing());
    } else {
        joints.push_back(m_distTierod[0]);
        joints.push_back(m_distTierod[1]);
    }
    database.WriteJoints(joints);
    database.WriteBodyBodyLoads(bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
