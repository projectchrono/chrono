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
// Authors: Daniel Melanz
// =============================================================================
//
// Base class for a MacPherson strut modeled with bodies and constraints.
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
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChPointPointShape.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChMacPhersonStrut.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChMacPhersonStrut::m_pointNames[] = {"SPINDLE ", "UPRIGHT ", "LCA_F   ", "LCA_B   ",
                                                       "LCA_U   ", "LCA_CM  ", "SHOCK_C ", "SHOCK_A ",
                                                       "SPRING_C", "SPRING_A", "TIEROD_C", "TIEROD_U"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChMacPhersonStrut::ChMacPhersonStrut(const std::string& name) : ChSuspension(name) {}

ChMacPhersonStrut::~ChMacPhersonStrut() {
    auto sys = m_upright[0]->GetSystem();
    if (sys) {
        for (int i = 0; i < 2; i++) {
            sys->Remove(m_upright[i]);
            sys->Remove(m_strut[i]);
            sys->Remove(m_LCA[i]);

            sys->Remove(m_cylindricalStrut[i]);
            sys->Remove(m_universalStrut[i]);

            ChChassis::RemoveJoint(m_revoluteLCA[i]);
            ChChassis::RemoveJoint(m_sphericalLCA[i]);

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
void ChMacPhersonStrut::Initialize(std::shared_ptr<ChChassis> chassis,
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
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, right_ang_vel);
}

void ChMacPhersonStrut::InitializeSide(VehicleSide side,
                                       std::shared_ptr<ChChassis> chassis,
                                       std::shared_ptr<ChBody> tierod_body,
                                       const std::vector<ChVector<>>& points,
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

    // Create and initialize Strut body.
    m_strut[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_strut[side]->SetNameString(m_name + "_Strut" + suffix);
    m_strut[side]->SetPos((points[SPRING_C] + points[SPRING_U]) / 2);
    m_strut[side]->SetRot(chassisRot);
    m_strut[side]->SetMass(getStrutMass());
    m_strut[side]->SetInertiaXX(getStrutInertia());
    chassis->GetSystem()->AddBody(m_strut[side]);

    // Create and initialize Lower Control Arm body.
    // Determine the rotation matrix of the LCA, based on the plane of the hard points
    // (z axis normal to the plane of the LCA)
    w = Vcross(points[LCA_B] - points[LCA_U], points[LCA_F] - points[LCA_U]);
    w.Normalize();
    u = points[LCA_F] - points[LCA_B];
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_LCA[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_LCA[side]->SetNameString(m_name + "_LCA" + suffix);
    m_LCA[side]->SetPos(points[LCA_CM]);
    m_LCA[side]->SetRot(rot);
    m_LCA[side]->SetMass(getLCAMass());
    m_LCA[side]->SetInertiaXX(getLCAInertia());
    chassis->GetSystem()->AddBody(m_LCA[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(-CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the cylindrical joint between upright and strut.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the UCA.
    w = points[SPRING_C] - points[SPRING_U];
    w.Normalize();
    v = Vcross(points[SPRING_U] - points[LCA_B], points[SPRING_C] - points[LCA_B]);
    v.Normalize();
    u = Vcross(v, w);
    u.Normalize();
    rot.Set_A_axis(u, v, w);

    m_cylindricalStrut[side] = chrono_types::make_shared<ChLinkLockCylindrical>();
    m_cylindricalStrut[side]->SetNameString(m_name + "_cylindricalStrut" + suffix);
    m_cylindricalStrut[side]->Initialize(m_strut[side], m_upright[side],
                                         ChCoordsys<>(points[SPRING_U], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_cylindricalStrut[side]);

    // Create and initialize the universal joint between chassis and UCA.
    //// Determine the joint orientation matrix from the hardpoint locations by
    //// constructing a rotation matrix with the z axis along the joint direction
    //// and the y axis normal to the plane of the UCA.
    // v = Vcross(points[UCA_B] - points[UCA_U], points[UCA_F] - points[UCA_U]);
    // v.Normalize();
    // w = points[UCA_F] - points[UCA_B];
    // w.Normalize();
    // u = Vcross(v, w);
    // rot.Set_A_axis(u, v, w);
    // TODO: Is this the correct rotation matrix?
    m_universalStrut[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalStrut[side]->SetNameString(m_name + "_universalStrut" + suffix);
    m_universalStrut[side]->Initialize(chassis->GetBody(), m_strut[side],
                                       ChFrame<>(points[SPRING_C], rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_universalStrut[side]);

    // Create and initialize the revolute joint between chassis and LCA.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction
    // and the y axis normal to the plane of the LCA.
    v = Vcross(points[LCA_B] - points[LCA_U], points[LCA_F] - points[LCA_U]);
    v.Normalize();
    w = points[LCA_F] - points[LCA_B];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_revoluteLCA[side] = chrono_types::make_shared<ChVehicleJoint>(
        ChVehicleJoint::Type::REVOLUTE, m_name + "_revoluteLCA" + suffix, chassis->GetBody(), m_LCA[side],
        ChCoordsys<>((points[LCA_F] + points[LCA_B]) / 2, rot.Get_A_quaternion()), getLCABushingData());
    chassis->AddJoint(m_revoluteLCA[side]);

    // Create and initialize the spherical joint between upright and LCA.
    m_sphericalLCA[side] =
        chrono_types::make_shared<ChVehicleJoint>(ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalLCA" + suffix,
                                                  m_LCA[side], m_upright[side], ChCoordsys<>(points[LCA_U], QUNIT));
    chassis->AddJoint(m_sphericalLCA[side]);

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
    m_shock[side]->Initialize(chassis->GetBody(), m_upright[side], false, points[SHOCK_C], points[SHOCK_U]);
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis->GetBody(), m_upright[side], false, points[SPRING_C], points[SPRING_U]);
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

void ChMacPhersonStrut::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getStrutMass() + getLCAMass() + getUprightMass());
}

void ChMacPhersonStrut::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), m_spindle[LEFT]->GetMass(),
                           m_spindle[LEFT]->GetInertia());
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), m_spindle[RIGHT]->GetMass(),
                           m_spindle[RIGHT]->GetInertia());
    composite.AddComponent(m_strut[LEFT]->GetFrame_COG_to_abs(), m_strut[LEFT]->GetMass(), m_strut[LEFT]->GetInertia());
    composite.AddComponent(m_strut[RIGHT]->GetFrame_COG_to_abs(), m_strut[RIGHT]->GetMass(),
                           m_strut[RIGHT]->GetInertia());
    composite.AddComponent(m_LCA[LEFT]->GetFrame_COG_to_abs(), m_LCA[LEFT]->GetMass(), m_LCA[LEFT]->GetInertia());
    composite.AddComponent(m_LCA[RIGHT]->GetFrame_COG_to_abs(), m_LCA[RIGHT]->GetMass(), m_LCA[RIGHT]->GetInertia());
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
double ChMacPhersonStrut::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
ChSuspension::Force ChMacPhersonStrut::ReportSuspensionForce(VehicleSide side) const {
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
void ChMacPhersonStrut::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteLCA[side]->GetConstraintViolation();
        GetLog() << "LCA revolute          ";
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
        ChVectorDynamic<> C = m_sphericalLCA[side]->GetConstraintViolation();
        GetLog() << "LCA spherical         ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }

    // Universal joints
    {
        ChVectorDynamic<> C = m_universalStrut[side]->GetConstraintViolation();
        GetLog() << "Strut universal       ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
        GetLog() << "  " << C(3) << "\n";
    }

    // Cylindrical joints
    {
        ChVectorDynamic<> C = m_cylindricalStrut[side]->GetConstraintViolation();
        GetLog() << "Strut cylindrical     ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
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
void ChMacPhersonStrut::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationUpright(m_upright[LEFT], 0.5 * (m_pointsL[SPINDLE] + m_pointsL[UPRIGHT]), m_pointsL[SPRING_U],
                            m_pointsL[LCA_U], m_pointsL[TIEROD_U], getUprightRadius());
    AddVisualizationUpright(m_upright[RIGHT], 0.5 * (m_pointsR[SPINDLE] + m_pointsR[UPRIGHT]), m_pointsR[SPRING_U],
                            m_pointsR[LCA_U], m_pointsR[TIEROD_U], getUprightRadius());

    AddVisualizationStrut(m_strut[LEFT], m_pointsL[SPRING_C], m_pointsL[SPRING_U], getStrutRadius());
    AddVisualizationStrut(m_strut[RIGHT], m_pointsR[SPRING_C], m_pointsR[SPRING_U], getStrutRadius());

    AddVisualizationControlArm(m_LCA[LEFT], m_pointsL[LCA_F], m_pointsL[LCA_B], m_pointsL[LCA_U], getLCARadius());
    AddVisualizationControlArm(m_LCA[RIGHT], m_pointsR[LCA_F], m_pointsR[LCA_B], m_pointsR[LCA_U], getLCARadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());

    // Add visualization for the tie-rods
    if (UseTierodBodies()) {
        AddVisualizationTierod(m_tierod[LEFT], m_pointsL[TIEROD_C], m_pointsL[TIEROD_U], getTierodRadius());
        AddVisualizationTierod(m_tierod[RIGHT], m_pointsR[TIEROD_C], m_pointsR[TIEROD_U], getTierodRadius());
    } else {
        m_distTierod[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
        m_distTierod[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
    }
}

void ChMacPhersonStrut::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_upright[LEFT]);
    ChPart::RemoveVisualizationAssets(m_upright[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_strut[LEFT]);
    ChPart::RemoveVisualizationAssets(m_strut[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_LCA[LEFT]);
    ChPart::RemoveVisualizationAssets(m_LCA[RIGHT]);

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
void ChMacPhersonStrut::AddVisualizationStrut(std::shared_ptr<ChBody> strut,
                                              const ChVector<> pt_c,
                                              const ChVector<> pt_u,
                                              double radius) {
    // Express hardpoint locations in body frame.
    ChVector<> p_c = strut->TransformPointParentToLocal(pt_c);
    ChVector<> p_u = strut->TransformPointParentToLocal(pt_u);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_c;
    cyl->GetCylinderGeometry().p2 = p_u;
    cyl->GetCylinderGeometry().rad = radius;
    strut->AddVisualShape(cyl);
}

void ChMacPhersonStrut::AddVisualizationControlArm(std::shared_ptr<ChBody> arm,
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
    arm->AddVisualShape(cyl_F);

    auto cyl_B = chrono_types::make_shared<ChCylinderShape>();
    cyl_B->GetCylinderGeometry().p1 = p_B;
    cyl_B->GetCylinderGeometry().p2 = p_U;
    cyl_B->GetCylinderGeometry().rad = radius;
    arm->AddVisualShape(cyl_B);
}

void ChMacPhersonStrut::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
                                                const ChVector<> pt_C,
                                                const ChVector<> pt_U,
                                                const ChVector<> pt_L,
                                                const ChVector<> pt_T,
                                                double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_C = upright->TransformPointParentToLocal(pt_C);
    ChVector<> p_U = upright->TransformPointParentToLocal(pt_U);
    ChVector<> p_L = upright->TransformPointParentToLocal(pt_L);
    ChVector<> p_T = upright->TransformPointParentToLocal(pt_T);

    if ((p_L - p_C).Length2() > threshold2) {
        auto cyl_L = chrono_types::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_L;
        cyl_L->GetCylinderGeometry().p2 = p_C;
        cyl_L->GetCylinderGeometry().rad = radius;
        upright->AddVisualShape(cyl_L);
    }

    if ((p_U - p_C).Length2() > threshold2) {
        auto cyl_U = chrono_types::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = p_C;
        cyl_U->GetCylinderGeometry().rad = radius;
        upright->AddVisualShape(cyl_U);
    }

    if ((p_T - p_C).Length2() > threshold2) {
        auto cyl_T = chrono_types::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = p_C;
        cyl_T->GetCylinderGeometry().rad = radius;
        upright->AddVisualShape(cyl_T);
    }
}

void ChMacPhersonStrut::AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
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
    tierod->AddVisualShape(cyl);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_strut[0]);
    bodies.push_back(m_strut[1]);
    bodies.push_back(m_LCA[0]);
    bodies.push_back(m_LCA[1]);
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
    joints.push_back(m_cylindricalStrut[0]);
    joints.push_back(m_cylindricalStrut[1]);
    joints.push_back(m_universalStrut[0]);
    joints.push_back(m_universalStrut[1]);
    m_revoluteLCA[0]->IsKinematic() ? joints.push_back(m_revoluteLCA[0]->GetAsLink())
                                    : bushings.push_back(m_revoluteLCA[0]->GetAsBushing());
    m_revoluteLCA[1]->IsKinematic() ? joints.push_back(m_revoluteLCA[1]->GetAsLink())
                                    : bushings.push_back(m_revoluteLCA[1]->GetAsBushing());
    m_sphericalLCA[0]->IsKinematic() ? joints.push_back(m_sphericalLCA[0]->GetAsLink())
                                     : bushings.push_back(m_sphericalLCA[0]->GetAsBushing());
    m_sphericalLCA[1]->IsKinematic() ? joints.push_back(m_sphericalLCA[1]->GetAsLink())
                                     : bushings.push_back(m_sphericalLCA[1]->GetAsBushing());
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

void ChMacPhersonStrut::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_upright[0]);
    bodies.push_back(m_upright[1]);
    bodies.push_back(m_strut[0]);
    bodies.push_back(m_strut[1]);
    bodies.push_back(m_LCA[0]);
    bodies.push_back(m_LCA[1]);
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
    joints.push_back(m_cylindricalStrut[0]);
    joints.push_back(m_cylindricalStrut[1]);
    joints.push_back(m_universalStrut[0]);
    joints.push_back(m_universalStrut[1]);
    m_revoluteLCA[0]->IsKinematic() ? joints.push_back(m_revoluteLCA[0]->GetAsLink())
                                    : bushings.push_back(m_revoluteLCA[0]->GetAsBushing());
    m_revoluteLCA[1]->IsKinematic() ? joints.push_back(m_revoluteLCA[1]->GetAsLink())
                                    : bushings.push_back(m_revoluteLCA[1]->GetAsBushing());
    m_sphericalLCA[0]->IsKinematic() ? joints.push_back(m_sphericalLCA[0]->GetAsLink())
                                     : bushings.push_back(m_sphericalLCA[0]->GetAsBushing());
    m_sphericalLCA[1]->IsKinematic() ? joints.push_back(m_sphericalLCA[1]->GetAsLink())
                                     : bushings.push_back(m_sphericalLCA[1]->GetAsBushing());
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
