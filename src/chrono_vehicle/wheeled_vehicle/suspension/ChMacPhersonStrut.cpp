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

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

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
    if (!m_initialized)
        return;

    auto sys = m_upright[0]->GetSystem();
    if (!sys)
        return;

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

// -----------------------------------------------------------------------------
void ChMacPhersonStrut::Construct(std::shared_ptr<ChChassis> chassis,
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

void ChMacPhersonStrut::InitializeSide(VehicleSide side,
                                       std::shared_ptr<ChChassis> chassis,
                                       std::shared_ptr<ChBody> tierod_body,
                                       const std::vector<ChVector3d>& points,
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

    // Initialize spindle body (same orientation as the chassis)
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(spindleRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());

    // Create and initialize upright body (same orientation as the chassis)
    m_upright[side] = chrono_types::make_shared<ChBody>();
    m_upright[side]->SetName(m_name + "_upright" + suffix);
    m_upright[side]->SetTag(m_obj_tag);
    m_upright[side]->SetPos(points[UPRIGHT]);
    m_upright[side]->SetRot(chassisRot);
    m_upright[side]->SetMass(getUprightMass());
    m_upright[side]->SetInertiaXX(getUprightInertia());
    chassis->GetSystem()->AddBody(m_upright[side]);

    // Create and initialize Strut body.
    m_strut[side] = chrono_types::make_shared<ChBody>();
    m_strut[side]->SetName(m_name + "_Strut" + suffix);
    m_strut[side]->SetTag(m_obj_tag);
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
    rot.SetFromDirectionAxes(u, v, w);

    m_LCA[side] = chrono_types::make_shared<ChBody>();
    m_LCA[side]->SetName(m_name + "_LCA" + suffix);
    m_LCA[side]->SetTag(m_obj_tag);
    m_LCA[side]->SetPos(points[LCA_CM]);
    m_LCA[side]->SetRot(rot);
    m_LCA[side]->SetMass(getLCAMass());
    m_LCA[side]->SetInertiaXX(getLCAInertia());
    chassis->GetSystem()->AddBody(m_LCA[side]);

    // Create and initialize the revolute joint between upright and spindle.
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->SetTag(m_obj_tag);
    m_revolute[side]->Initialize(m_spindle[side], m_upright[side],
                                 ChFrame<>(points[SPINDLE], spindleRot * QuatFromAngleX(-CH_PI_2)));
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
    rot.SetFromDirectionAxes(u, v, w);

    m_cylindricalStrut[side] = chrono_types::make_shared<ChLinkLockCylindrical>();
    m_cylindricalStrut[side]->SetName(m_name + "_cylindricalStrut" + suffix);
    m_cylindricalStrut[side]->SetTag(m_obj_tag);
    m_cylindricalStrut[side]->Initialize(m_strut[side], m_upright[side],
                                         ChFrame<>(points[SPRING_U], rot.GetQuaternion()));
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
    // rot.SetFromDirectionAxes(u, v, w);
    // TODO: Is this the correct rotation matrix?
    m_universalStrut[side] = chrono_types::make_shared<ChLinkUniversal>();
    m_universalStrut[side]->SetName(m_name + "_universalStrut" + suffix);
    m_universalStrut[side]->SetTag(m_obj_tag);
    m_universalStrut[side]->Initialize(chassis->GetBody(), m_strut[side],
                                       ChFrame<>(points[SPRING_C], rot.GetQuaternion()));
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
    rot.SetFromDirectionAxes(u, v, w);

    m_revoluteLCA[side] = chrono_types::make_shared<ChJoint>(
        ChJoint::Type::REVOLUTE, m_name + "_revoluteLCA" + suffix, chassis->GetBody(), m_LCA[side],
        ChFrame<>((points[LCA_F] + points[LCA_B]) / 2, rot.GetQuaternion()), getLCABushingData());
    m_revoluteLCA[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_revoluteLCA[side]);

    // Create and initialize the spherical joint between upright and LCA.
    m_sphericalLCA[side] =
        chrono_types::make_shared<ChJoint>(ChJoint::Type::SPHERICAL, m_name + "_sphericalLCA" + suffix,
                                                  m_LCA[side], m_upright[side], ChFrame<>(points[LCA_U], QUNIT));
    m_sphericalLCA[side]->SetTag(m_obj_tag);
    chassis->AddJoint(m_sphericalLCA[side]);

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

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetName(m_name + "_shock" + suffix);
    m_shock[side]->SetTag(m_obj_tag);
    m_shock[side]->Initialize(chassis->GetBody(), m_upright[side], false, points[SHOCK_C], points[SHOCK_U]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetName(m_name + "_spring" + suffix);
    m_spring[side]->SetTag(m_obj_tag);
    m_spring[side]->Initialize(chassis->GetBody(), m_upright[side], false, points[SPRING_C], points[SPRING_U]);
    m_spring[side]->SetRestLength(getSpringRestLength());
    m_spring[side]->RegisterForceFunctor(getSpringForceFunctor());
    chassis->GetSystem()->AddLink(m_spring[side]);

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
    m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector3d(0, -1, 0));
    chassis->GetSystem()->Add(m_axle_to_spindle[side]);
}

void ChMacPhersonStrut::InitializeInertiaProperties() {
    m_mass = 2 * (getSpindleMass() + getStrutMass() + getLCAMass() + getUprightMass());
    if (UseTierodBodies()) {
        m_mass += 2 * getTierodMass();
    }
}

void ChMacPhersonStrut::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaStruct(getStrutInertia());
    ChMatrix33<> inertiaLCA(getLCAInertia());
    ChMatrix33<> inertiaUpright(getUprightInertia());

    CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_strut[LEFT]->GetFrameCOMToAbs(), getStrutMass(), inertiaStruct);
    composite.AddComponent(m_strut[RIGHT]->GetFrameCOMToAbs(), getStrutMass(), inertiaStruct);
    composite.AddComponent(m_LCA[LEFT]->GetFrameCOMToAbs(), getLCAMass(), inertiaLCA);
    composite.AddComponent(m_LCA[RIGHT]->GetFrameCOMToAbs(), getLCAMass(), inertiaLCA);
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
double ChMacPhersonStrut::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChMacPhersonStrut::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChMacPhersonStrut::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteLCA[side]->GetConstraintViolation();
        std::cout << "LCA revolute          ";
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
        ChVectorDynamic<> C = m_sphericalLCA[side]->GetConstraintViolation();
        std::cout << "LCA spherical         ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }

    // Universal joints
    {
        ChVectorDynamic<> C = m_universalStrut[side]->GetConstraintViolation();
        std::cout << "Strut universal       ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
        std::cout << "  " << C(3) << "\n";
    }

    // Cylindrical joints
    {
        ChVectorDynamic<> C = m_cylindricalStrut[side]->GetConstraintViolation();
        std::cout << "Strut cylindrical     ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
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
                                              const ChVector3d pt_c,
                                              const ChVector3d pt_u,
                                              double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_c = strut->TransformPointParentToLocal(pt_c);
    ChVector3d p_u = strut->TransformPointParentToLocal(pt_u);

    utils::ChBodyGeometry::AddVisualizationCylinder(strut, p_c, p_u, radius);
}

void ChMacPhersonStrut::AddVisualizationControlArm(std::shared_ptr<ChBody> arm,
                                                   const ChVector3d pt_F,
                                                   const ChVector3d pt_B,
                                                   const ChVector3d pt_U,
                                                   double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_F = arm->TransformPointParentToLocal(pt_F);
    ChVector3d p_B = arm->TransformPointParentToLocal(pt_B);
    ChVector3d p_U = arm->TransformPointParentToLocal(pt_U);

    utils::ChBodyGeometry::AddVisualizationCylinder(arm, p_F, p_U, radius);
    utils::ChBodyGeometry::AddVisualizationCylinder(arm, p_B, p_U, radius);
}

void ChMacPhersonStrut::AddVisualizationUpright(std::shared_ptr<ChBody> upright,
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

void ChMacPhersonStrut::AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
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
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

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
    ExportJointList(jsonDocument, joints);
    ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChMacPhersonStrut::Output(ChOutput& database) const {
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
    database.WriteBodyBodyLoads(bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
