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
// Base class for a Pitman Arm steering subsystem.
// Derived from ChSteering, but still an abstract base class.
//
// =============================================================================

#include <vector>

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/steering/ChPitmanArmShafts.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
ChPitmanArmShafts::ChPitmanArmShafts(const std::string& name, bool vehicle_frame_inertia, bool rigid_column)
    : ChSteering(name), m_vehicle_frame_inertia(vehicle_frame_inertia), m_rigid(rigid_column) {}

ChPitmanArmShafts::~ChPitmanArmShafts() {
    auto sys = m_arm->GetSystem();
    if (sys) {
        sys->Remove(m_arm);
        sys->Remove(m_revolute);
        sys->Remove(m_revsph);
        sys->Remove(m_universal);
        sys->Remove(m_shaft_A);
        sys->Remove(m_shaft_C);
        sys->Remove(m_shaft_A1);
        sys->Remove(m_shaft_C1);
        sys->Remove(m_shaft_arm);
        sys->Remove(m_shaft_chassis);
        sys->Remove(m_shaft_gear);
        sys->Remove(m_shaft_motor);
        sys->Remove(m_rigid_connection);
        sys->Remove(m_spring_connection);
    }
}

// -----------------------------------------------------------------------------
void ChPitmanArmShafts::Construct(std::shared_ptr<ChChassis> chassis,
                                  const ChVector3d& location,
                                  const ChQuaternion<>& rotation) {
    auto chassisBody = chassis->GetBody();
    auto sys = chassisBody->GetSystem();

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassisBody->GetFrameRefToAbs().GetRot();

    // Express the steering reference frame in the absolute coordinate system.
    ChFrame<> steering_to_abs(location, rotation);
    steering_to_abs.ConcatenatePreTransformation(chassisBody->GetFrameRefToAbs());

    // Transform all points and directions to absolute frame.
    std::vector<ChVector3d> points(NUM_POINTS);
    std::vector<ChVector3d> dirs(NUM_DIRS);

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d rel_pos = getLocation(static_cast<PointId>(i));
        points[i] = steering_to_abs.TransformPointLocalToParent(rel_pos);
    }

    for (int i = 0; i < NUM_DIRS; i++) {
        ChVector3d rel_dir = getDirection(static_cast<DirectionId>(i));
        dirs[i] = steering_to_abs.TransformDirectionLocalToParent(rel_dir);
    }

    // Unit vectors for orientation matrices.
    ChVector3d u;
    ChVector3d v;
    ChVector3d w;
    ChMatrix33<> rot;

    // Create and initialize the steering link body
    m_link = chrono_types::make_shared<ChBody>();
    m_link->SetName(m_name + "_link");
    m_link->SetTag(m_obj_tag);
    m_link->SetPos(points[STEERINGLINK]);
    m_link->SetRot(steering_to_abs.GetRot());
    m_link->SetMass(getSteeringLinkMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia = TransformInertiaMatrix(getSteeringLinkInertiaMoments(), getSteeringLinkInertiaProducts(),
                                                      chassisRot, steering_to_abs.GetRot());
        m_link->SetInertia(inertia);
    } else {
        m_link->SetInertiaXX(getSteeringLinkInertiaMoments());
        m_link->SetInertiaXY(getSteeringLinkInertiaProducts());
    }
    sys->AddBody(m_link);

    m_pP = m_link->TransformPointParentToLocal(points[UNIV]);
    m_pI = m_link->TransformPointParentToLocal(points[REVSPH_S]);
    m_pTP = m_link->TransformPointParentToLocal(points[TIEROD_PA]);
    m_pTI = m_link->TransformPointParentToLocal(points[TIEROD_IA]);

    // Create and initialize the Pitman arm body
    m_arm = chrono_types::make_shared<ChBody>();
    m_arm->SetName(m_name + "_arm");
    m_arm->SetTag(m_obj_tag);
    m_arm->SetPos(points[PITMANARM]);
    m_arm->SetRot(steering_to_abs.GetRot());
    m_arm->SetMass(getPitmanArmMass());
    if (m_vehicle_frame_inertia) {
        ChMatrix33<> inertia = TransformInertiaMatrix(getPitmanArmInertiaMoments(), getPitmanArmInertiaProducts(),
                                                      chassisRot, steering_to_abs.GetRot());
        m_arm->SetInertia(inertia);
    } else {
        m_arm->SetInertiaXX(getPitmanArmInertiaMoments());
        m_arm->SetInertiaXY(getPitmanArmInertiaProducts());
    }
    sys->AddBody(m_arm);

    // Cache points for arm visualization (expressed in the arm frame)
    m_pC = m_arm->TransformPointParentToLocal(points[REV]);
    m_pL = m_arm->TransformPointParentToLocal(points[UNIV]);

    // Create and initialize the revolute joint between chassis and Pitman arm.
    // The z direction of the joint orientation matrix is dirs[REV_AXIS], assumed
    // to be a unit vector.
    u = points[PITMANARM] - points[REV];
    v = Vcross(dirs[REV_AXIS], u);
    v.Normalize();
    u = Vcross(v, dirs[REV_AXIS]);
    rot.SetFromDirectionAxes(u, v, dirs[REV_AXIS]);

    m_revolute = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute->SetName(m_name + "_revolute");
    m_revolute->SetTag(m_obj_tag);
    m_revolute->Initialize(chassisBody, m_arm, ChFrame<>(points[REV], rot.GetQuaternion()));
    sys->AddLink(m_revolute);

    // Create and initialize the universal joint between the Pitman arm and steering link.
    // The x and y directions of the joint orientation matrix are given by
    // dirs[UNIV_AXIS_ARM] and dirs[UNIV_AXIS_LINK], assumed to be unit vectors
    // and orthogonal.
    w = Vcross(dirs[UNIV_AXIS_ARM], dirs[UNIV_AXIS_LINK]);
    rot.SetFromDirectionAxes(dirs[UNIV_AXIS_ARM], dirs[UNIV_AXIS_LINK], w);

    m_universal = chrono_types::make_shared<ChLinkUniversal>();
    m_universal->SetName(m_name + "_universal");
    m_universal->SetTag(m_obj_tag);
    m_universal->Initialize(m_arm, m_link, ChFrame<>(points[UNIV], rot.GetQuaternion()));
    sys->AddLink(m_universal);

    // Create and initialize the revolute-spherical joint (massless idler arm).
    // The length of the idler arm is the distance between the two hardpoints.
    // The z direction of the revolute joint orientation matrix is
    // dirs[REVSPH_AXIS], assumed to be a unit vector.
    double distance = (points[REVSPH_S] - points[REVSPH_R]).Length();

    u = points[REVSPH_S] - points[REVSPH_R];
    v = Vcross(dirs[REVSPH_AXIS], u);
    v.Normalize();
    u = Vcross(v, dirs[REVSPH_AXIS]);
    rot.SetFromDirectionAxes(u, v, dirs[REVSPH_AXIS]);

    m_revsph = chrono_types::make_shared<ChLinkRevoluteSpherical>();
    m_revsph->SetName(m_name + "_revsph");
    m_revsph->SetTag(m_obj_tag);
    m_revsph->Initialize(chassisBody, m_link, ChCoordsys<>(points[REVSPH_R], rot.GetQuaternion()), distance);
    sys->AddLink(m_revsph);

    //// TODO: Decide if shaftC should be attached to chassis or if it should be "fixed"
    ////       Right now: fixed.

    // Create all shafts in steering column
    //    Chassis --X-- shaftC --M-- shaftC1 --S-- shaftA1 --G-- shaftA --X-- Arm
    // All shafts are aligned with dir[REV_AXIS], assumed to be a unit vector.
    double inertia = getSteeringColumnInertia();

    m_shaft_C = chrono_types::make_shared<ChShaft>();
    m_shaft_C->SetName(m_name + "_shaftC");
    m_shaft_C->SetTag(m_obj_tag);
    m_shaft_C->SetInertia(inertia);
    m_shaft_C->SetFixed(true);
    sys->AddShaft(m_shaft_C);

    m_shaft_C1 = chrono_types::make_shared<ChShaft>();
    m_shaft_C1->SetName(m_name + "_shaftC1");
    m_shaft_C1->SetTag(m_obj_tag);
    m_shaft_C1->SetInertia(inertia);
    sys->AddShaft(m_shaft_C1);

    m_shaft_A1 = chrono_types::make_shared<ChShaft>();
    m_shaft_A1->SetName(m_name + "_shaftA1");
    m_shaft_A1->SetTag(m_obj_tag);
    m_shaft_A1->SetInertia(inertia);
    sys->AddShaft(m_shaft_A1);

    m_shaft_A = chrono_types::make_shared<ChShaft>();
    m_shaft_A->SetName(m_name + "_shaftA");
    m_shaft_A->SetTag(m_obj_tag);
    m_shaft_A->SetInertia(inertia);
    sys->AddShaft(m_shaft_A);

    // Rigidly attach shaftA to the arm body
    m_shaft_arm = chrono_types::make_shared<ChShaftBodyRotation>();
    m_shaft_arm->SetName(m_name + "_shaftA_to_arm");
    m_shaft_arm->Initialize(m_shaft_A, m_arm, dirs[REV_AXIS]);
    sys->Add(m_shaft_arm);

    // Rigidly attach shaftC to the chassis body
    ////m_shaft_chassis = chrono_types::make_shared<ChShaftBodyRotation>();
    ////m_shaft_chassis->SetName(m_name + "_shaftC_to_chassis");
    ////m_shaft_chassis->Initialize(m_shaft_C, chassisBody, dirs[REV_AXIS]);
    ////sys->Add(m_shaft_chassis);

    // A motor (for steering input) between shaftC and shaftC1
    // The setpoint for the motor angle function is set in Synchronize()
    m_shaft_motor = chrono_types::make_shared<ChShaftsMotorPosition>();
    m_shaft_motor->SetName(m_name + "_motor");
    m_shaft_motor->Initialize(m_shaft_C, m_shaft_C1);
    auto motor_fun = chrono_types::make_shared<ChFunctionSetpoint>();
    m_shaft_motor->SetPositionFunction(motor_fun);
    sys->Add(m_shaft_motor);

    // A reduction gear between shaftA and shaftA1
    // (note order of connected shafts for gear_ratio > 1)
    m_shaft_gear = chrono_types::make_shared<ChShaftsGear>();
    m_shaft_gear->SetName(m_name + "_transmission");
    m_shaft_gear->SetTransmissionRatio(getGearRatio());
    m_shaft_gear->Initialize(m_shaft_A, m_shaft_A1);
    sys->Add(m_shaft_gear);

    // Connect shaftA1 and shaftC1 (compliant or rigid connection)
    if (m_rigid) {
        // Use a gear couple with ratio=1
        m_rigid_connection = chrono_types::make_shared<ChShaftsGear>();
        m_rigid_connection->SetName(m_name + "_rigid_column");
        m_rigid_connection->SetTransmissionRatio(1.0);
        m_rigid_connection->Initialize(m_shaft_A1, m_shaft_C1);
        sys->Add(m_rigid_connection);
    } else {
        // Use a torsional spring between shaftA1 and shaftC1
        m_spring_connection = chrono_types::make_shared<ChShaftsTorsionSpring>();
        m_spring_connection->SetName(m_name + "_compliant_column");
        m_spring_connection->SetTorsionalStiffness(getSteeringCompliance());
        ////m_spring_connection->SetTorsionalDamping(10);
        m_spring_connection->Initialize(m_shaft_C1, m_shaft_A1);
        sys->Add(m_spring_connection);
    }
}

// -----------------------------------------------------------------------------
void ChPitmanArmShafts::Synchronize(double time, const DriverInputs& driver_inputs) {
    auto fun = std::static_pointer_cast<ChFunctionSetpoint>(m_shaft_motor->GetPositionFunction());
    fun->SetSetpoint(getMaxAngle() * driver_inputs.m_steering, time);
}

void ChPitmanArmShafts::InitializeInertiaProperties() {
    m_mass = getSteeringLinkMass() + getPitmanArmMass();
}

void ChPitmanArmShafts::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(m_rel_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;
    composite.AddComponent(m_link->GetFrameCOMToAbs(), m_link->GetMass(), m_link->GetInertia());
    composite.AddComponent(m_arm->GetFrameCOMToAbs(), m_arm->GetMass(), m_arm->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(m_xform.GetRot());

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------
void ChPitmanArmShafts::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    // Visualization for link
    utils::ChBodyGeometry::AddVisualizationCylinder(m_link, m_pP, m_pI, getSteeringLinkRadius());
    utils::ChBodyGeometry::AddVisualizationCylinder(m_link, m_pP, m_pTP, getSteeringLinkRadius());
    utils::ChBodyGeometry::AddVisualizationCylinder(m_link, m_pI, m_pTI, getSteeringLinkRadius());

    // Visualization for arm
    utils::ChBodyGeometry::AddVisualizationCylinder(m_arm, m_pC, m_pL, getPitmanArmRadius());

    // Visualization for rev-sph link
    m_revsph->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChPitmanArmShafts::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_link);
    ChPart::RemoveVisualizationAssets(m_arm);
    ChPart::RemoveVisualizationAssets(m_revsph);
}

// -----------------------------------------------------------------------------
void ChPitmanArmShafts::LogConstraintViolations() {
    // Revolute joint
    {
        ChVectorDynamic<> C = m_revolute->GetConstraintViolation();
        std::cout << "Revolute              ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    // Universal joint
    {
        ChVectorDynamic<> C = m_universal->GetConstraintViolation();
        std::cout << "Universal             ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "\n";
    }

    // Revolute-spherical joint
    {
        ChVectorDynamic<> C = m_revsph->GetConstraintViolation();
        std::cout << "Revolute-spherical    ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "\n";
    }

    //// TODO
    //// Constraint violations for the various shaft couples
}

void ChPitmanArmShafts::GetShaftInformation(double time,
                                            double& motor_input,
                                            double& motor_input_der,
                                            std::vector<double>& shaft_angles,
                                            std::vector<double>& shaft_velocities,
                                            std::vector<double>& constraint_violations,
                                            ChVector3d& arm_angular_vel) const {
    auto fun = std::static_pointer_cast<ChFunctionSetpoint>(m_shaft_motor->GetPositionFunction());
    motor_input = fun->GetVal(time);
    motor_input_der = fun->GetDer(time);

    shaft_angles.push_back(m_shaft_C->GetPos());
    shaft_angles.push_back(m_shaft_C1->GetPos());
    shaft_angles.push_back(m_shaft_A1->GetPos());
    shaft_angles.push_back(m_shaft_A->GetPos());

    shaft_velocities.push_back(m_shaft_C->GetPosDt());
    shaft_velocities.push_back(m_shaft_C1->GetPosDt());
    shaft_velocities.push_back(m_shaft_A1->GetPosDt());
    shaft_velocities.push_back(m_shaft_A->GetPosDt());

    constraint_violations.push_back(m_shaft_motor->GetConstraintViolation());
    constraint_violations.push_back(m_shaft_gear->GetConstraintViolation());
    if (m_rigid)
        constraint_violations.push_back(m_rigid_connection->GetConstraintViolation());

    arm_angular_vel = m_arm->GetAngVelLocal();
}

// -----------------------------------------------------------------------------
void ChPitmanArmShafts::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    bodies.push_back(m_arm);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_shaft_C);
    shafts.push_back(m_shaft_C1);
    shafts.push_back(m_shaft_A1);
    shafts.push_back(m_shaft_A);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_revsph);
    joints.push_back(m_universal);
    ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChShaftsCouple>> couples;
    couples.push_back(m_shaft_motor);
    couples.push_back(m_shaft_gear);
    if (m_rigid)
        couples.push_back(m_rigid_connection);
    else
        couples.push_back(m_spring_connection);
    ExportCouplesList(jsonDocument, couples);
}

void ChPitmanArmShafts::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_link);
    bodies.push_back(m_arm);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_shaft_C);
    shafts.push_back(m_shaft_C1);
    shafts.push_back(m_shaft_A1);
    shafts.push_back(m_shaft_A);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute);
    joints.push_back(m_revsph);
    joints.push_back(m_universal);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChShaftsCouple>> couples;
    couples.push_back(m_shaft_motor);
    couples.push_back(m_shaft_gear);
    if (m_rigid)
        couples.push_back(m_rigid_connection);
    else
        couples.push_back(m_spring_connection);
    database.WriteCouples(couples);
}

}  // end namespace vehicle
}  // end namespace chrono
