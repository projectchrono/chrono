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
// Authors: Radu Serban, Justin Madsen
// =============================================================================
//
// Base class for a double-A arm suspension modeled with distance constraints.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChDoubleWishboneReduced.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChDoubleWishboneReduced::ChDoubleWishboneReduced(const std::string& name) : ChSuspension(name) {}

ChDoubleWishboneReduced::~ChDoubleWishboneReduced() {
    if (!m_initialized)
        return;

    auto sys = m_upright[0]->GetSystem();
    if (!sys)
        return;

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_upright[i]);
        sys->Remove(m_distUCA_F[i]);
        sys->Remove(m_distUCA_B[i]);
        sys->Remove(m_distLCA_F[i]);
        sys->Remove(m_distLCA_B[i]);
        sys->Remove(m_distTierod[i]);
        sys->Remove(m_shock[i]);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::Initialize(std::shared_ptr<ChChassis> chassis,
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
    InitializeSide(LEFT, chassis->GetBody(), tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), tierod_body, m_pointsR, right_ang_vel);
}

void ChDoubleWishboneReduced::InitializeSide(VehicleSide side,
                                             std::shared_ptr<ChBodyAuxRef> chassis,
                                             std::shared_ptr<ChBody> tierod_body,
                                             const std::vector<ChVector3d>& points,
                                             double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrameRefToAbs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * QuatFromAngleZ(sign * getToeAngle()) * QuatFromAngleX(sign * getCamberAngle());

    // Create and initialize spindle body
    m_spindle[side] = chrono_types::make_shared<ChBody>();
    m_spindle[side]->SetName(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize upright body (same orientation as the chassis)
    m_upright[side] = chrono_types::make_shared<ChBody>();
    m_upright[side]->SetName(m_name + "_upright" + suffix);
    m_upright[side]->SetPos(points[UPRIGHT]);
    m_upright[side]->SetRot(chassisRot);
    m_upright[side]->SetMass(getUprightMass());
    m_upright[side]->SetInertiaXX(getUprightInertia());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Create and initialize joints
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side],
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    m_distUCA_F[side] = chrono_types::make_shared<ChLinkDistance>();
    m_distUCA_F[side]->SetName(m_name + "_distUCA_F" + suffix);
    m_distUCA_F[side]->Initialize(chassis, m_upright[side], false, points[UCA_F], points[UCA_U]);
    chassis->GetSystem()->AddLink(m_distUCA_F[side]);

    m_distUCA_B[side] = chrono_types::make_shared<ChLinkDistance>();
    m_distUCA_B[side]->SetName(m_name + "_distUCA_B" + suffix);
    m_distUCA_B[side]->Initialize(chassis, m_upright[side], false, points[UCA_B], points[UCA_U]);
    chassis->GetSystem()->AddLink(m_distUCA_B[side]);

    m_distLCA_F[side] = chrono_types::make_shared<ChLinkDistance>();
    m_distLCA_F[side]->SetName(m_name + "_distLCA_F" + suffix);
    m_distLCA_F[side]->Initialize(chassis, m_upright[side], false, points[LCA_F], points[LCA_U]);
    chassis->GetSystem()->AddLink(m_distLCA_F[side]);

    m_distLCA_B[side] = chrono_types::make_shared<ChLinkDistance>();
    m_distLCA_B[side]->SetName(m_name + "_distLCA_B" + suffix);
    m_distLCA_B[side]->Initialize(chassis, m_upright[side], false, points[LCA_B], points[LCA_U]);
    chassis->GetSystem()->AddLink(m_distLCA_B[side]);

    m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
    m_distTierod[side]->SetName(m_name + "_distTierod" + suffix);
    m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
    chassis->GetSystem()->AddLink(m_distTierod[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_upright[side], false, points[SHOCK_C], points[SHOCK_U]);
    m_shock[side]->SetRestLength(getSpringRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    // Create and initialize the axle shaft and its connection to the spindle.
    // Note that the spindle rotates about the Y axis.
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

void ChDoubleWishboneReduced::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getUprightMass());
}

void ChDoubleWishboneReduced::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaUpright(getUprightInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_upright[LEFT]->GetFrameCOMToAbs(), getUprightMass(), inertiaUpright);
    composite.AddComponent(m_upright[RIGHT]->GetFrameCOMToAbs(), getUprightMass(), inertiaUpright);

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChDoubleWishboneReduced::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChDoubleWishboneReduced::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(1);

    forces[0] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    // Add visualization for uprights
    AddVisualizationUpright(m_upright[LEFT], 0.5 * (m_pointsL[SPINDLE] + m_pointsL[UPRIGHT]), m_pointsL[UCA_U],
                            m_pointsL[LCA_U], m_pointsL[TIEROD_U], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], 0.5 * (m_pointsR[SPINDLE] + m_pointsR[UPRIGHT]), m_pointsR[UCA_U],
                            m_pointsR[LCA_U], m_pointsR[TIEROD_U], getUprightRadius());

    // Add visualization for the spring-dampers
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));

    // Add visualization for the arm and tie-rod distance constraints
    m_distTierod[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_distTierod[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    m_distUCA_F[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_distUCA_F[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    m_distUCA_B[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_distUCA_B[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    m_distLCA_F[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_distLCA_F[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    m_distLCA_B[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    m_distLCA_B[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChDoubleWishboneReduced::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_upright[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upright[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_distTierod[LEFT]);
    ChPart::RemoveVisualizationAssets(m_distTierod[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_distUCA_F[LEFT]);
    ChPart::RemoveVisualizationAssets(m_distUCA_F[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_distUCA_B[LEFT]);
    ChPart::RemoveVisualizationAssets(m_distUCA_B[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_distLCA_F[LEFT]);
    ChPart::RemoveVisualizationAssets(m_distLCA_F[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_distLCA_B[LEFT]);
    ChPart::RemoveVisualizationAssets(m_distLCA_B[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                                      const ChVector3d pt_C,
                                                      const ChVector3d pt_U,
                                                      const ChVector3d pt_L,
                                                      const ChVector3d pt_T,
                                                      double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector3d p_C = upright->TransformPointParentToLocal(pt_C);
    ChVector3d p_U = upright->TransformPointParentToLocal(pt_U);
    ChVector3d p_L = upright->TransformPointParentToLocal(pt_L);
    ChVector3d p_T = upright->TransformPointParentToLocal(pt_T);

    if ((p_L - p_C).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(upright, p_L, p_C, radius);
    }

    if ((p_U - p_C).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(upright, p_U, p_C, radius);
    }

    if ((p_T - p_C).Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(upright, p_T, p_C, radius);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::LogConstraintViolations(VehicleSide side) {
    // Revolute joint
    {
        ChVectorDynamic<> C = m_revolute[side]->GetConstraintViolation();
        std::cout << "Spindle revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    // Distance constraints
    std::cout << "UCA front distance    ";
    std::cout << "  " << m_distUCA_F[side]->GetCurrentDistance() - m_distUCA_F[side]->GetImposedDistance() << "\n";

    std::cout << "UCA back distance     ";
    std::cout << "  " << m_distUCA_B[side]->GetCurrentDistance() - m_distUCA_B[side]->GetImposedDistance() << "\n";

    std::cout << "LCA front distance    ";
    std::cout << "  " << m_distLCA_F[side]->GetCurrentDistance() - m_distLCA_F[side]->GetImposedDistance() << "\n";

    std::cout << "LCA back distance     ";
    std::cout << "  " << m_distLCA_B[side]->GetCurrentDistance() - m_distLCA_B[side]->GetImposedDistance() << "\n";

    std::cout << "Tierod distance       ";
    std::cout << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChDoubleWishboneReduced::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_distUCA_F[0]);
    joints.push_back(m_distUCA_F[1]);
    joints.push_back(m_distUCA_B[0]);
    joints.push_back(m_distUCA_B[1]);
    joints.push_back(m_distLCA_F[0]);
    joints.push_back(m_distLCA_F[1]);
    joints.push_back(m_distLCA_B[0]);
    joints.push_back(m_distLCA_B[1]);
    joints.push_back(m_distTierod[0]);
    joints.push_back(m_distTierod[1]);
    ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChDoubleWishboneReduced::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_distUCA_F[0]);
    joints.push_back(m_distUCA_F[1]);
    joints.push_back(m_distUCA_B[0]);
    joints.push_back(m_distUCA_B[1]);
    joints.push_back(m_distLCA_F[0]);
    joints.push_back(m_distLCA_F[1]);
    joints.push_back(m_distLCA_B[0]);
    joints.push_back(m_distLCA_B[1]);
    joints.push_back(m_distTierod[0]);
    joints.push_back(m_distTierod[1]);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
