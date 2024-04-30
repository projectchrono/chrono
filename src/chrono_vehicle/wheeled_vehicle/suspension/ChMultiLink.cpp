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
// Base class for a multi-link suspension modeled with bodies and constraints.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChMultiLink.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChMultiLink::m_pointNames[] = {"SPINDLE  ", "UPRIGHT  ", "UA_F     ", "UA_B     ", "UA_U     ",
                                                 "UA_CM    ", "LAT_C    ", "LAT_U    ", "LAT_CM   ", "TL_C     ",
                                                 "TL_U     ", "TL_CM    ", "SHOCK_C  ", "SHOCK_L  ", "SPRING_C ",
                                                 "SPRING_L ", "TIEROD_C ", "TIEROD_U "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChMultiLink::ChMultiLink(const std::string& name) : ChSuspension(name) {}

ChMultiLink::~ChMultiLink() {
    if (!m_initialized)
        return;

    auto sys = m_upright[0]->GetSystem();
    if (!sys)
        return;

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_upright[i]);
        sys->Remove(m_upperArm[i]);
        sys->Remove(m_lateral[i]);
        sys->Remove(m_trailingLink[i]);

        sys->Remove(m_revoluteUA[i]);
        sys->Remove(m_sphericalUA[i]);
        sys->Remove(m_universalLateralChassis[i]);
        sys->Remove(m_sphericalLateralUpright[i]);
        sys->Remove(m_universalTLChassis[i]);
        sys->Remove(m_sphericalTLUpright[i]);

        if (m_tierod[i]) {
            sys->Remove(m_tierod[i]);
            ChChassis::RemoveJoint(m_sphericalTierod[i]);
            ChChassis::RemoveJoint(m_universalTierod[i]);
        }
        if (m_distTierod[i]) {
            sys->Remove(m_distTierod[i]);
        }

        sys->Remove(m_shock[i]);
        sys->Remove(m_spring[i]);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::Initialize(std::shared_ptr<ChChassis> chassis,
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

    // Transform all points and directions to absolute frame
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
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, m_dirsR, right_ang_vel);
}

void ChMultiLink::InitializeSide(VehicleSide side,
                                 std::shared_ptr<ChChassis> chassis,
                                 std::shared_ptr<ChBody> tierod_body,
                                 const std::vector<ChVector3d>& points,
                                 const std::vector<ChVector3d>& dirs,
                                 double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrameRefToAbs().GetRot();

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

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

    // Create and initialize upright body (same orientation as the chassis)
    m_upright[side] = chrono_types::make_shared<ChBody>();
    m_upright[side]->SetName(m_name + "_upright" + suffix);
    m_upright[side]->SetPos(points[UPRIGHT]);
    m_upright[side]->SetRot(chassisRot);
    m_upright[side]->SetMass(getUprightMass());
    m_upright[side]->SetInertiaXX(getUprightInertia());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Create and initialize Upper Arm body.
    // Determine the rotation matrix of the upper arm based on the plane of the hard points
    // (z axis normal to the plane of the upper arm)
    w = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
    w.Normalize();
    u = points[UA_F] - points[UA_B];
    u.Normalize();
    v = Vcross(w, u);
    rot.SetFromDirectionAxes(u, v, w);

    m_upperArm[side] = chrono_types::make_shared<ChBody>();
    m_upperArm[side]->SetName(m_name + "_upperArm" + suffix);
    m_upperArm[side]->SetPos(points[UA_CM]);
    m_upperArm[side]->SetRot(rot);
    m_upperArm[side]->SetMass(getUpperArmMass());
    m_upperArm[side]->SetInertiaXX(getUpperArmInertia());
    chassis->GetSystem()->AddBody(m_upperArm[side]);

    // Create and initialize lateral body.
    // Determine the rotation matrix of the lateral based on the plane of the hard points
    // (z-axis along the length of the track rod)
    v = Vcross(points[LAT_U] - points[TL_U], points[LAT_C] - points[TL_U]);
    v.Normalize();
    w = points[LAT_C] - points[LAT_U];
    w.Normalize();
    u = Vcross(v, w);
    rot.SetFromDirectionAxes(u, v, w);

    m_lateral[side] = chrono_types::make_shared<ChBody>();
    m_lateral[side]->SetName(m_name + "_lateral" + suffix);
    m_lateral[side]->SetPos(points[LAT_CM]);
    m_lateral[side]->SetRot(rot);
    m_lateral[side]->SetMass(getLateralMass());
    m_lateral[side]->SetInertiaXX(getLateralInertia());
    chassis->GetSystem()->AddBody(m_lateral[side]);

    // Create and initialize trailing link body.
    // Determine the rotation matrix of the trailing link based on the plane of the hard points
    // (z-axis along the length of the trailing link)
    v = Vcross(points[TL_U] - points[SPRING_L], points[TL_C] - points[SPRING_L]);
    v.Normalize();
    w = points[TL_C] - points[TL_U];
    w.Normalize();
    u = Vcross(v, w);
    rot.SetFromDirectionAxes(u, v, w);

    m_trailingLink[side] = chrono_types::make_shared<ChBody>();
    m_trailingLink[side]->SetName(m_name + "_trailingLink" + suffix);
    m_trailingLink[side]->SetPos(points[TL_CM]);
    m_trailingLink[side]->SetRot(rot);
    m_trailingLink[side]->SetMass(getTrailingLinkMass());
    m_trailingLink[side]->SetInertiaXX(getTrailingLinkInertia());
    chassis->GetSystem()->AddBody(m_trailingLink[side]);

    // Create and initialize the revolute joint between upright and spindle.
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side],
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the revolute joint between chassis and upper arm.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the upper arm.
    v = Vcross(points[UA_B] - points[UA_U], points[UA_F] - points[UA_U]);
    v.Normalize();
    w = points[UA_F] - points[UA_B];
    w.Normalize();
    u = Vcross(v, w);
    rot.SetFromDirectionAxes(u, v, w);

    m_revoluteUA[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteUA[side]->SetName(m_name + "_revoluteUA" + suffix);
    m_revoluteUA[side]->Initialize(chassis->GetBody(), m_upperArm[side],
                                   ChFrame<>((points[UA_F] + points[UA_B]) / 2, rot.GetQuaternion()));
    chassis->GetSystem()->AddLink(m_revoluteUA[side]);

    // Create and initialize the spherical joint between upright and upper arm.
    m_sphericalUA[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalUA[side]->SetName(m_name + "_sphericalUA" + suffix);
    m_sphericalUA[side]->Initialize(m_upperArm[side], m_upright[side], ChFrame<>(points[UA_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUA[side]);

    // Create and initialize the spherical joint between upright and track rod.
    m_sphericalLateralUpright[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalLateralUpright[side]->SetName(m_name + "_sphericalLateralUpright" + suffix);
    m_sphericalLateralUpright[side]->Initialize(m_lateral[side], m_upright[side], ChFrame<>(points[LAT_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLateralUpright[side]);

    // Create and initialize the universal joint between chassis and track rod.
    u = dirs[UNIV_AXIS_CHASSIS_LAT];
    v = dirs[UNIV_AXIS_LINK_LAT];
    w = Vcross(u, v);
    rot.SetFromDirectionAxes(u, v, w);

    m_universalLateralChassis[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalLateralChassis[side]->SetName(m_name + "_universalLateralChassis" + suffix);
    m_universalLateralChassis[side]->Initialize(m_lateral[side], chassis->GetBody(),
                                                ChFrame<>(points[LAT_C], rot.GetQuaternion()));
    chassis->GetSystem()->AddLink(m_universalLateralChassis[side]);

    // Create and initialize the spherical joint between upright and trailing link.
    m_sphericalTLUpright[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalTLUpright[side]->SetName(m_name + "_sphericalTLUpright" + suffix);
    m_sphericalTLUpright[side]->Initialize(m_trailingLink[side], m_upright[side], ChFrame<>(points[TL_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTLUpright[side]);

    // Create and initialize the universal joint between chassis and trailing link.
    u = dirs[UNIV_AXIS_CHASSIS_TL];
    v = dirs[UNIV_AXIS_LINK_TL];
    w = Vcross(u, v);
    rot.SetFromDirectionAxes(u, v, w);

    m_universalTLChassis[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalTLChassis[side]->SetName(m_name + "_universalTLChassis" + suffix);
    m_universalTLChassis[side]->Initialize(m_trailingLink[side], chassis->GetBody(),
                                           ChFrame<>(points[TL_C], rot.GetQuaternion()));
    chassis->GetSystem()->AddLink(m_universalTLChassis[side]);

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
        m_tierod[side]->SetPos((points[TIEROD_U] + points[TIEROD_C]) / 2);
        m_tierod[side]->SetRot(rot.GetQuaternion());
        m_tierod[side]->SetMass(getTierodMass());
        m_tierod[side]->SetInertiaXX(getTierodInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_tierod[side]);

        // Connect tierod body to upright (spherical) and chassis (universal)
        m_sphericalTierod[side] = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalTierod" + suffix, m_upright[side], m_tierod[side],
            ChFrame<>(points[TIEROD_U], QUNIT), getTierodBushingData());
        chassis->AddJoint(m_sphericalTierod[side]);
        m_universalTierod[side] = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::UNIVERSAL, m_name + "_universalTierod" + suffix, tierod_body, m_tierod[side],
            ChFrame<>(points[TIEROD_C], rot.GetQuaternion()), getTierodBushingData());
        chassis->AddJoint(m_universalTierod[side]);
    } else {
        // Create and initialize the tierod distance constraint between chassis and upright.
        m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
        m_distTierod[side]->SetName(m_name + "_distTierod" + suffix);
        m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
        chassis->GetSystem()->AddLink(m_distTierod[side]);
    }

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis->GetBody(), m_trailingLink[side], false, points[SHOCK_C], points[SHOCK_L]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetName(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis->GetBody(), m_trailingLink[side], false, points[SPRING_C], points[SPRING_L]);
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

void ChMultiLink::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getUpperArmMass() + getUpperArmMass() + getTrailingLinkMass() + getUprightMass());
    if (UseTierodBodies()) {
        m_mass += 2 * getTierodMass();
    }
}

void ChMultiLink::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaUpperArm(getUpperArmInertia());
    ChMatrix33<> inertiaLateral(getLateralInertia());
    ChMatrix33<> inertiaTrailingLink(getTrailingLinkInertia());
    ChMatrix33<> inertiaUpright(getUprightInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);

    composite.AddComponent(m_upperArm[LEFT]->GetFrameCOMToAbs(), getUpperArmMass(), inertiaUpperArm);
    composite.AddComponent(m_upperArm[RIGHT]->GetFrameCOMToAbs(), getUpperArmMass(), inertiaUpperArm);

    composite.AddComponent(m_lateral[LEFT]->GetFrameCOMToAbs(), getUpperArmMass(), inertiaLateral);
    composite.AddComponent(m_lateral[RIGHT]->GetFrameCOMToAbs(), getUpperArmMass(), inertiaLateral);

    composite.AddComponent(m_trailingLink[LEFT]->GetFrameCOMToAbs(), getTrailingLinkMass(), inertiaTrailingLink);
    composite.AddComponent(m_trailingLink[RIGHT]->GetFrameCOMToAbs(), getTrailingLinkMass(), inertiaTrailingLink);

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
double ChMultiLink::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChMultiLink::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteUA[side]->GetConstraintViolation();
        std::cout << "Upper arm revolute    ";
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

    // Spherical joints
    {
        ChVectorDynamic<> C = m_sphericalUA[side]->GetConstraintViolation();
        std::cout << "Upper arm spherical   ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalLateralUpright[side]->GetConstraintViolation();
        std::cout << "Lateral-Upright spherical  ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalTLUpright[side]->GetConstraintViolation();
        std::cout << "TL-Upright spherical  ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }

    // Universal joints
    {
        ChVectorDynamic<> C = m_universalLateralChassis[side]->GetConstraintViolation();
        std::cout << "Lateral-Chassis universal  ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalTLChassis[side]->GetConstraintViolation();
        std::cout << "TL-Chassis universal  ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
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
void ChMultiLink::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationUpright(m_upright[LEFT], m_pointsL[UA_U], m_pointsL[LAT_U], m_pointsL[TL_U], m_pointsL[TIEROD_U],
                            m_pointsL[UPRIGHT], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], m_pointsR[UA_U], m_pointsR[LAT_U], m_pointsR[TL_U], m_pointsR[TIEROD_U],
                            m_pointsR[UPRIGHT], getUprightRadius());

    AddVisualizationUpperArm(m_upperArm[LEFT], m_pointsL[UA_F], m_pointsL[UA_B], m_pointsL[UA_U], getUpperArmRadius());
    AddVisualizationUpperArm(m_upperArm[RIGHT], m_pointsR[UA_F], m_pointsR[UA_B], m_pointsR[UA_U], getUpperArmRadius());

    AddVisualizationLateral(m_lateral[LEFT], m_pointsL[LAT_U], m_pointsL[LAT_C], getLateralRadius());
    AddVisualizationLateral(m_lateral[RIGHT], m_pointsR[LAT_U], m_pointsR[LAT_C], getLateralRadius());

    AddVisualizationTrailingLink(m_trailingLink[LEFT], m_pointsL[TL_C], m_pointsL[SPRING_L], m_pointsL[TL_U],
                                 getTrailingLinkRadius());
    AddVisualizationTrailingLink(m_trailingLink[RIGHT], m_pointsR[TL_C], m_pointsR[SPRING_L], m_pointsR[TL_U],
                                 getTrailingLinkRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
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

void ChMultiLink::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_upright[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upright[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_upperArm[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upperArm[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_lateral[LEFT]);
    ChPart::RemoveVisualizationAssets(m_lateral[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_trailingLink[LEFT]);
    ChPart::RemoveVisualizationAssets(m_trailingLink[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

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
void ChMultiLink::AddVisualizationUpperArm(std::shared_ptr<ChBody> arm,
                                           const ChVector3d pt_F,
                                           const ChVector3d pt_B,
                                           const ChVector3d pt_U,
                                           double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_F = arm->TransformPointParentToLocal(pt_F);
    ChVector3d p_B = arm->TransformPointParentToLocal(pt_B);
    ChVector3d p_U = arm->TransformPointParentToLocal(pt_U);

    ChVehicleGeometry::AddVisualizationCylinder(arm, p_F, p_U, radius);
    ChVehicleGeometry::AddVisualizationCylinder(arm, p_B, p_U, radius);
}

void ChMultiLink::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                          const ChVector3d pt_UA,
                                          const ChVector3d pt_TR,
                                          const ChVector3d pt_TL,
                                          const ChVector3d pt_T,
                                          const ChVector3d pt_U,
                                          double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector3d p_UA = upright->TransformPointParentToLocal(pt_UA);
    ChVector3d p_TR = upright->TransformPointParentToLocal(pt_TR);
    ChVector3d p_TL = upright->TransformPointParentToLocal(pt_TL);
    ChVector3d p_T = upright->TransformPointParentToLocal(pt_T);
    ChVector3d p_U = upright->TransformPointParentToLocal(pt_U);

    if (p_UA.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(upright, p_UA, VNULL, radius);
    }

    if (p_TR.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(upright, p_TR, VNULL, radius);
    }

    if (p_TL.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(upright, p_TL, VNULL, radius);
    }

    if (p_T.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(upright, p_T, VNULL, radius);
    }

    if (p_U.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(upright, p_U, VNULL, radius);
    }
}

void ChMultiLink::AddVisualizationLateral(std::shared_ptr<ChBody> rod,
                                          const ChVector3d pt_C,
                                          const ChVector3d pt_U,
                                          double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_C = rod->TransformPointParentToLocal(pt_C);
    ChVector3d p_U = rod->TransformPointParentToLocal(pt_U);

    ChVehicleGeometry::AddVisualizationCylinder(rod, p_C, p_U, radius);
}

void ChMultiLink::AddVisualizationTrailingLink(std::shared_ptr<ChBody> link,
                                               const ChVector3d pt_C,
                                               const ChVector3d pt_S,
                                               const ChVector3d pt_U,
                                               double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_C = link->TransformPointParentToLocal(pt_C);
    ChVector3d p_S = link->TransformPointParentToLocal(pt_S);
    ChVector3d p_U = link->TransformPointParentToLocal(pt_U);

    ChVehicleGeometry::AddVisualizationCylinder(link, p_C, p_S, radius);
    ChVehicleGeometry::AddVisualizationCylinder(link, p_S, p_U, radius);
}

void ChMultiLink::AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
                                         const ChVector3d pt_C,
                                         const ChVector3d pt_U,
                                         double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_C = tierod->TransformPointParentToLocal(pt_C);
    ChVector3d p_U = tierod->TransformPointParentToLocal(pt_U);

    ChVehicleGeometry::AddVisualizationCylinder(tierod, p_C, p_U, radius);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_upperArm[0]);
    bodies.push_back(m_upperArm[1]);
    bodies.push_back(m_lateral[0]);
    bodies.push_back(m_lateral[1]);
    bodies.push_back(m_trailingLink[0]);
    bodies.push_back(m_trailingLink[1]);
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
    joints.push_back(m_revoluteUA[0]);
    joints.push_back(m_revoluteUA[1]);
    joints.push_back(m_sphericalUA[0]);
    joints.push_back(m_sphericalUA[1]);
    joints.push_back(m_universalLateralChassis[0]);
    joints.push_back(m_universalLateralChassis[1]);
    joints.push_back(m_sphericalLateralUpright[0]);
    joints.push_back(m_sphericalLateralUpright[1]);
    joints.push_back(m_universalTLChassis[0]);
    joints.push_back(m_universalTLChassis[1]);
    joints.push_back(m_sphericalTLUpright[0]);
    joints.push_back(m_sphericalTLUpright[1]);
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
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChMultiLink::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_upperArm[0]);
    bodies.push_back(m_upperArm[1]);
    bodies.push_back(m_lateral[0]);
    bodies.push_back(m_lateral[1]);
    bodies.push_back(m_trailingLink[0]);
    bodies.push_back(m_trailingLink[1]);
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
    joints.push_back(m_revoluteUA[0]);
    joints.push_back(m_revoluteUA[1]);
    joints.push_back(m_sphericalUA[0]);
    joints.push_back(m_sphericalUA[1]);
    joints.push_back(m_universalLateralChassis[0]);
    joints.push_back(m_universalLateralChassis[1]);
    joints.push_back(m_sphericalLateralUpright[0]);
    joints.push_back(m_sphericalLateralUpright[1]);
    joints.push_back(m_universalTLChassis[0]);
    joints.push_back(m_universalTLChassis[1]);
    joints.push_back(m_sphericalTLUpright[0]);
    joints.push_back(m_sphericalTLUpright[1]);
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
