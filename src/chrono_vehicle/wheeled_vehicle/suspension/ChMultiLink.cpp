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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/assets/ChColorAsset.h"

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
    auto sys = m_upright[0]->GetSystem();
    if (sys) {
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
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::Initialize(std::shared_ptr<ChChassis> chassis,
                             std::shared_ptr<ChSubchassis> subchassis,
                             std::shared_ptr<ChSteering> steering,
                             const ChVector<>& location,
                             double left_ang_vel,
                             double right_ang_vel) {
    m_parent = chassis;
    m_rel_loc = location;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Transform all points and directions to absolute frame
    m_pointsL.resize(NUM_POINTS);
    m_pointsR.resize(NUM_POINTS);

    m_dirsL.resize(NUM_DIRS);
    m_dirsR.resize(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> rel_pos = getLocation(static_cast<PointId>(i));
        m_pointsL[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
        rel_pos.y() = -rel_pos.y();
        m_pointsR[i] = suspension_to_abs.TransformLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector<> rel_dir = getDirection(static_cast<DirectionId>(i));
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
                                 const std::vector<ChVector<>>& points,
                                 const std::vector<ChVector<>>& dirs,
                                 double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrame_REF_to_abs().GetRot();

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize upright body (same orientation as the chassis)
    m_upright[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upright[side]->SetNameString(m_name + "_upright" + suffix);
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
    rot.Set_A_axis(u, v, w);

    m_upperArm[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_upperArm[side]->SetNameString(m_name + "_upperArm" + suffix);
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
    rot.Set_A_axis(u, v, w);

    m_lateral[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_lateral[side]->SetNameString(m_name + "_lateral" + suffix);
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
    rot.Set_A_axis(u, v, w);

    m_trailingLink[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_trailingLink[side]->SetNameString(m_name + "_trailingLink" + suffix);
    m_trailingLink[side]->SetPos(points[TL_CM]);
    m_trailingLink[side]->SetRot(rot);
    m_trailingLink[side]->SetMass(getTrailingLinkMass());
    m_trailingLink[side]->SetInertiaXX(getTrailingLinkInertia());
    chassis->GetSystem()->AddBody(m_trailingLink[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));

    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
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
    rot.Set_A_axis(u, v, w);

    m_revoluteUA[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteUA[side]->SetNameString(m_name + "_revoluteUA" + suffix);
    m_revoluteUA[side]->Initialize(chassis->GetBody(), m_upperArm[side],
                                   ChCoordsys<>((points[UA_F] + points[UA_B]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteUA[side]);

    // Create and initialize the spherical joint between upright and upper arm.
    m_sphericalUA[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalUA[side]->SetNameString(m_name + "_sphericalUA" + suffix);
    m_sphericalUA[side]->Initialize(m_upperArm[side], m_upright[side], ChCoordsys<>(points[UA_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalUA[side]);

    // Create and initialize the spherical joint between upright and track rod.
    m_sphericalLateralUpright[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalLateralUpright[side]->SetNameString(m_name + "_sphericalLateralUpright" + suffix);
    m_sphericalLateralUpright[side]->Initialize(m_lateral[side], m_upright[side], ChCoordsys<>(points[LAT_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLateralUpright[side]);

    // Create and initialize the universal joint between chassis and track rod.
    u = dirs[UNIV_AXIS_CHASSIS_LAT];
    v = dirs[UNIV_AXIS_LINK_LAT];
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_universalLateralChassis[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalLateralChassis[side]->SetNameString(m_name + "_universalLateralChassis" + suffix);
    m_universalLateralChassis[side]->Initialize(m_lateral[side], chassis->GetBody(),
                                                ChFrame<>(points[LAT_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalLateralChassis[side]);

    // Create and initialize the spherical joint between upright and trailing link.
    m_sphericalTLUpright[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalTLUpright[side]->SetNameString(m_name + "_sphericalTLUpright" + suffix);
    m_sphericalTLUpright[side]->Initialize(m_trailingLink[side], m_upright[side], ChCoordsys<>(points[TL_U], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTLUpright[side]);

    // Create and initialize the universal joint between chassis and trailing link.
    u = dirs[UNIV_AXIS_CHASSIS_TL];
    v = dirs[UNIV_AXIS_LINK_TL];
    w = Vcross(u, v);
    rot.Set_A_axis(u, v, w);

    m_universalTLChassis[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalTLChassis[side]->SetNameString(m_name + "_universalTLChassis" + suffix);
    m_universalTLChassis[side]->Initialize(m_trailingLink[side], chassis->GetBody(),
                                           ChFrame<>(points[TL_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalTLChassis[side]);

    if (UseTierodBodies()) {
        // Orientation of tierod body
        w = (points[TIEROD_U] - points[TIEROD_C]).GetNormalized();
        u = chassisRot.GetXaxis();
        v = Vcross(w, u).GetNormalized();
        u = Vcross(v, w);
        rot.Set_A_axis(u, v, w);

        // Create the tierod body
        m_tierod[side] = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
        m_tierod[side]->SetNameString(m_name + "_tierodBody" + suffix);
        m_tierod[side]->SetPos((points[TIEROD_U] + points[TIEROD_C]) / 2);
        m_tierod[side]->SetRot(rot.Get_A_quaternion());
        m_tierod[side]->SetMass(getTierodMass());
        m_tierod[side]->SetInertiaXX(getTierodInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_tierod[side]);

        // Connect tierod body to upright (spherical) and chassis (universal)
        m_sphericalTierod[side] = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalTierod" + suffix, m_upright[side], m_tierod[side],
            ChCoordsys<>(points[TIEROD_U], QUNIT), getTierodBushingData());
        chassis->AddJoint(m_sphericalTierod[side]);
        m_universalTierod[side] = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::UNIVERSAL, m_name + "_universalTierod" + suffix, tierod_body, m_tierod[side],
            ChCoordsys<>(points[TIEROD_C], rot.Get_A_quaternion()), getTierodBushingData());
        chassis->AddJoint(m_universalTierod[side]);
    } else {
        // Create and initialize the tierod distance constraint between chassis and upright.
        m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
        m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
        m_distTierod[side]->Initialize(tierod_body, m_upright[side], false, points[TIEROD_C], points[TIEROD_U]);
        chassis->GetSystem()->AddLink(m_distTierod[side]);
    }

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis->GetBody(), m_trailingLink[side], false, points[SHOCK_C], points[SHOCK_L]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis->GetBody(), m_trailingLink[side], false, points[SPRING_C], points[SPRING_L]);
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
}

void ChMultiLink::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getUpperArmMass() + getLateralMass() + getTrailingLinkMass() + getUprightMass());
}

void ChMultiLink::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), m_spindle[LEFT]->GetMass(),
                           m_spindle[LEFT]->GetInertia());
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), m_spindle[RIGHT]->GetMass(),
                           m_spindle[RIGHT]->GetInertia());

    composite.AddComponent(m_upperArm[LEFT]->GetFrame_COG_to_abs(), m_upperArm[LEFT]->GetMass(),
                           m_upperArm[LEFT]->GetInertia());
    composite.AddComponent(m_upperArm[RIGHT]->GetFrame_COG_to_abs(), m_upperArm[RIGHT]->GetMass(),
                           m_upperArm[RIGHT]->GetInertia());

    composite.AddComponent(m_lateral[LEFT]->GetFrame_COG_to_abs(), m_lateral[LEFT]->GetMass(),
                           m_lateral[LEFT]->GetInertia());
    composite.AddComponent(m_lateral[RIGHT]->GetFrame_COG_to_abs(), m_lateral[RIGHT]->GetMass(),
                           m_lateral[RIGHT]->GetInertia());

    composite.AddComponent(m_trailingLink[LEFT]->GetFrame_COG_to_abs(), m_trailingLink[LEFT]->GetMass(),
                           m_trailingLink[LEFT]->GetInertia());
    composite.AddComponent(m_trailingLink[RIGHT]->GetFrame_COG_to_abs(), m_trailingLink[RIGHT]->GetMass(),
                           m_trailingLink[RIGHT]->GetInertia());

    composite.AddComponent(m_upright[LEFT]->GetFrame_COG_to_abs(), m_upright[LEFT]->GetMass(),
                           m_upright[LEFT]->GetInertia());
    composite.AddComponent(m_upright[RIGHT]->GetFrame_COG_to_abs(), m_upright[RIGHT]->GetMass(),
                           m_upright[RIGHT]->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
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
ChSuspension::Force ChMultiLink::ReportSuspensionForce(VehicleSide side) const {
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
void ChMultiLink::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteUA[side]->GetConstraintViolation();
        GetLog() << "Upper arm revolute    ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revolute[side]->GetConstraintViolation();
        GetLog() << "Spindle revolute      ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }

    // Spherical joints
    {
        ChVectorDynamic<> C = m_sphericalUA[side]->GetConstraintViolation();
        GetLog() << "Upper arm spherical   ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalLateralUpright[side]->GetConstraintViolation();
        GetLog() << "Lateral-Upright spherical  ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalTLUpright[side]->GetConstraintViolation();
        GetLog() << "TL-Upright spherical  ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }

    // Universal joints
    {
        ChVectorDynamic<> C = m_universalLateralChassis[side]->GetConstraintViolation();
        GetLog() << "Lateral-Chassis universal  ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalTLChassis[side]->GetConstraintViolation();
        GetLog() << "TL-Chassis universal  ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }

    // Tierod constraint
    if (UseTierodBodies()) {
        {
            const auto& C = m_sphericalTierod[side]->GetConstraintViolation();
            GetLog() << "Tierod spherical      ";
            GetLog() << "  " << C(0) << "  ";
            GetLog() << "  " << C(1) << "  ";
            GetLog() << "  " << C(2) << "\n";
        }
        {
            const auto& C = m_universalTierod[side]->GetConstraintViolation();
            GetLog() << "Tierod universal      ";
            GetLog() << "  " << C(0) << "  ";
            GetLog() << "  " << C(1) << "  ";
            GetLog() << "  " << C(2) << "\n";
            GetLog() << "  " << C(3) << "\n";
        }
    } else {
        GetLog() << "Tierod distance       ";
        GetLog() << "  " << m_distTierod[side]->GetCurrentDistance() - m_distTierod[side]->GetImposedDistance() << "\n";
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
    m_spring[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.06, 150, 15));
    m_spring[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSpring>(0.06, 150, 15));

    m_shock[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
    m_shock[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());

    // Add visualization for the tie-rods
    if (UseTierodBodies()) {
        AddVisualizationTierod(m_tierod[LEFT], m_pointsL[TIEROD_C], m_pointsL[TIEROD_U], getTierodRadius());
        AddVisualizationTierod(m_tierod[RIGHT], m_pointsR[TIEROD_C], m_pointsR[TIEROD_U], getTierodRadius());
    } else {
        m_distTierod[LEFT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
        m_distTierod[RIGHT]->AddAsset(chrono_types::make_shared<ChPointPointSegment>());
        m_distTierod[LEFT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
        m_distTierod[RIGHT]->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
    }
}

void ChMultiLink::RemoveVisualizationAssets() {
    ChSuspension::RemoveVisualizationAssets();

    m_upright[LEFT]->GetAssets().clear();
    m_upright[RIGHT]->GetAssets().clear();

    m_upperArm[LEFT]->GetAssets().clear();
    m_upperArm[RIGHT]->GetAssets().clear();

    m_lateral[LEFT]->GetAssets().clear();
    m_lateral[RIGHT]->GetAssets().clear();

    m_trailingLink[LEFT]->GetAssets().clear();
    m_trailingLink[RIGHT]->GetAssets().clear();

    m_spring[LEFT]->GetAssets().clear();
    m_spring[RIGHT]->GetAssets().clear();

    m_shock[LEFT]->GetAssets().clear();
    m_shock[RIGHT]->GetAssets().clear();

    if (UseTierodBodies()) {
        m_tierod[LEFT]->GetAssets().clear();
        m_tierod[RIGHT]->GetAssets().clear();
    } else {
        m_distTierod[LEFT]->GetAssets().clear();
        m_distTierod[RIGHT]->GetAssets().clear();
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMultiLink::AddVisualizationUpperArm(std::shared_ptr<ChBody> arm,
                                           const ChVector<> pt_F,
                                           const ChVector<> pt_B,
                                           const ChVector<> pt_U,
                                           double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_F = arm->TransformPointParentToLocal(pt_F);
    ChVector<> p_B = arm->TransformPointParentToLocal(pt_B);
    ChVector<> p_U = arm->TransformPointParentToLocal(pt_U);

    auto cyl_F = chrono_types::make_shared<ChCylinderShape>();
    cyl_F->GetCylinderGeometry().p1 = p_F;
    cyl_F->GetCylinderGeometry().p2 = p_U;
    cyl_F->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_F);

    auto cyl_B = chrono_types::make_shared<ChCylinderShape>();
    cyl_B->GetCylinderGeometry().p1 = p_B;
    cyl_B->GetCylinderGeometry().p2 = p_U;
    cyl_B->GetCylinderGeometry().rad = radius;
    arm->AddAsset(cyl_B);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.6f, 0.2f, 0.6f));
    arm->AddAsset(col);
}

void ChMultiLink::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                          const ChVector<> pt_UA,
                                          const ChVector<> pt_TR,
                                          const ChVector<> pt_TL,
                                          const ChVector<> pt_T,
                                          const ChVector<> pt_U,
                                          double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_UA = upright->TransformPointParentToLocal(pt_UA);
    ChVector<> p_TR = upright->TransformPointParentToLocal(pt_TR);
    ChVector<> p_TL = upright->TransformPointParentToLocal(pt_TL);
    ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);
    ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);

    if (p_UA.Length2() > threshold2) {
        auto cyl_UA = chrono_types::make_shared<ChCylinderShape>();
        cyl_UA->GetCylinderGeometry().p1 = p_UA;
        cyl_UA->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_UA->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_UA);
    }

    if (p_TR.Length2() > threshold2) {
        auto cyl_TR = chrono_types::make_shared<ChCylinderShape>();
        cyl_TR->GetCylinderGeometry().p1 = p_TR;
        cyl_TR->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_TR->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_TR);
    }

    if (p_TL.Length2() > threshold2) {
        auto cyl_TL = chrono_types::make_shared<ChCylinderShape>();
        cyl_TL->GetCylinderGeometry().p1 = p_TL;
        cyl_TL->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_TL->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_TL);
    }

    if (p_T.Length2() > threshold2) {
        auto cyl_T = chrono_types::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_T->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_T);
    }

    if (p_U.Length2() > threshold2) {
        auto cyl_U = chrono_types::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_U->GetCylinderGeometry().rad = radius;
        upright->AddAsset(cyl_U);
    }

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    upright->AddAsset(col);
}

void ChMultiLink::AddVisualizationLateral(std::shared_ptr<ChBody> rod,
                                          const ChVector<> pt_C,
                                          const ChVector<> pt_U,
                                          double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = rod->TransformPointParentToLocal(pt_C);
    ChVector<> p_U = rod->TransformPointParentToLocal(pt_U);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_C;
    cyl->GetCylinderGeometry().p2 = p_U;
    cyl->GetCylinderGeometry().rad = radius;
    rod->AddAsset(cyl);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.6f, 0.2f));
    rod->AddAsset(col);
}

void ChMultiLink::AddVisualizationTrailingLink(std::shared_ptr<ChBody> link,
                                               const ChVector<> pt_C,
                                               const ChVector<> pt_S,
                                               const ChVector<> pt_U,
                                               double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = link->TransformPointParentToLocal(pt_C);
    ChVector<> p_S = link->TransformPointParentToLocal(pt_S);
    ChVector<> p_U = link->TransformPointParentToLocal(pt_U);

    auto cyl1 = chrono_types::make_shared<ChCylinderShape>();
    cyl1->GetCylinderGeometry().p1 = p_C;
    cyl1->GetCylinderGeometry().p2 = p_S;
    cyl1->GetCylinderGeometry().rad = radius;
    link->AddAsset(cyl1);

    auto cyl2 = chrono_types::make_shared<ChCylinderShape>();
    cyl2->GetCylinderGeometry().p1 = p_S;
    cyl2->GetCylinderGeometry().p2 = p_U;
    cyl2->GetCylinderGeometry().rad = radius;
    link->AddAsset(cyl2);

    auto col = chrono_types::make_shared<ChColorAsset>();
    col->SetColor(ChColor(0.2f, 0.6f, 0.6f));
    link->AddAsset(col);
}

void ChMultiLink::AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
                                         const ChVector<> pt_C,
                                         const ChVector<> pt_U,
                                         double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = tierod->TransformPointParentToLocal(pt_C);
    ChVector<> p_U = tierod->TransformPointParentToLocal(pt_U);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_C;
    cyl->GetCylinderGeometry().p2 = p_U;
    cyl->GetCylinderGeometry().rad = radius;
    tierod->AddAsset(cyl);

    tierod->AddAsset(chrono_types::make_shared<ChColorAsset>(0.8f, 0.3f, 0.3f));
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
    ChPart::ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ChPart::ExportShaftList(jsonDocument, shafts);

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
    ChPart::ExportJointList(jsonDocument, joints);
    ChPart::ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
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
