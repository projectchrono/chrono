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
// Authors: Radu Serban, Holger Haut
// =============================================================================
//
// Base class for a Hendrickson PRIMAXX EX suspension.
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
// TODO: connect transverse beam with universal joint?!?
//
// =============================================================================

#include "chrono/assets/ChVisualShapeCylinder.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChHendricksonPRIMAXX.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChHendricksonPRIMAXX::m_pointNames[] = {
    "SPINDLE ",     "KNUCKLE_L",   "KNUCKLE_U",    "TIEROD_C",     "TIEROD_K",    "TORQUEROD_C",
    "TORQUEROD_AH", "LOWERBEAM_C", "LOWERBEAM_AH", "LOWERBEAM_TB", "SHOCKAH_C",   "SHOCKAH_AH",
    "SHOCKLB_C",    "SHOCKLB_LB",  "KNUCKLE_CM",   "TORQUEROD_CM", "LOWERBEAM_CM"};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChHendricksonPRIMAXX::ChHendricksonPRIMAXX(const std::string& name) : ChSuspension(name) {}

ChHendricksonPRIMAXX::~ChHendricksonPRIMAXX() {
    if (!IsInitialized())
        return;

    auto sys = m_transversebeam->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_transversebeam);
    sys->Remove(m_axlehousing);

    for (int i = 0; i < 2; i++) {
        sys->Remove(m_knuckle[i]);
        sys->Remove(m_torquerod[i]);
        sys->Remove(m_lowerbeam[i]);
        sys->Remove(m_revoluteKingpin[i]);
        sys->Remove(m_sphericalTorquerod[i]);
        sys->Remove(m_revoluteTorquerod[i]);
        sys->Remove(m_sphericalLowerbeam[i]);
        sys->Remove(m_revoluteLowerbeam[i]);
        sys->Remove(m_sphericalTB[i]);

        if (m_tierod[i]) {
            sys->Remove(m_tierod[i]);
            ChChassis::RemoveJoint(m_sphericalTierod[i]);
            ChChassis::RemoveJoint(m_universalTierod[i]);
        }
        if (m_distTierod[i]) {
            sys->Remove(m_distTierod[i]);
        }

        sys->Remove(m_shockLB[i]);
        sys->Remove(m_shockAH[i]);
    }
}

// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::Construct(std::shared_ptr<ChChassis> chassis,
                                     std::shared_ptr<ChSubchassis> subchassis,
                                     std::shared_ptr<ChSteering> steering,
                                     const ChVector3d& location,
                                     double left_ang_vel,
                                     double right_ang_vel) {
    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Transform the location of the axle body COM to absolute frame.
    ChVector3d axleCOM_local = getAxlehousingCOM();
    ChVector3d axleCOM = suspension_to_abs.TransformPointLocalToParent(axleCOM_local);

    // Calculate end points on the axle body, expressed in the absolute frame
    // (for visualization)
    ChVector3d midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
    ChVector3d outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    m_outerL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_outerR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    // Create and initialize the axle housing body.
    m_axlehousing = chrono_types::make_shared<ChBody>();
    m_axlehousing->SetName(m_name + "_axlehousing");
    m_axlehousing->SetTag(m_obj_tag);
    m_axlehousing->SetPos(axleCOM);
    m_axlehousing->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_axlehousing->SetMass(getAxlehousingMass());
    m_axlehousing->SetInertiaXX(getAxlehousingInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axlehousing);

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

    // Create transverse beam body
    ChVector3d tbCOM_local = getTransversebeamCOM();
    ChVector3d tbCOM = suspension_to_abs.TransformPointLocalToParent(tbCOM_local);

    m_transversebeam = chrono_types::make_shared<ChBody>();
    m_transversebeam->SetName(m_name + "_transversebeam");
    m_transversebeam->SetTag(m_obj_tag);
    m_transversebeam->SetPos(tbCOM);
    m_transversebeam->SetRot(chassis->GetBody()->GetFrameRefToAbs().GetRot());
    m_transversebeam->SetMass(getTransversebeamMass());
    m_transversebeam->SetInertiaXX(getTransversebeamInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_transversebeam);

    // Initialize left and right sides.
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();
    InitializeSide(LEFT, chassis, tierod_body, m_pointsL, m_dirsL, left_ang_vel);
    InitializeSide(RIGHT, chassis, tierod_body, m_pointsR, m_dirsR, right_ang_vel);
}

void ChHendricksonPRIMAXX::InitializeSide(VehicleSide side,
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

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = chrono_types::make_shared<ChBody>();
    m_knuckle[side]->SetName(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetTag(m_obj_tag);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(chassisRot);
    m_knuckle[side]->SetMass(getKnuckleMass());
    m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
    chassis->GetSystem()->AddBody(m_knuckle[side]);

    // Initialize spindle body (same orientation as the chassis)
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());

    // Create and initialize torque rod body.
    // Determine the rotation matrix of the torque rod based on the plane of the hard points
    // (z-axis along the length of the torque rod)
    v = Vcross(points[TORQUEROD_AH] - points[LOWERBEAM_AH], points[TORQUEROD_C] - points[LOWERBEAM_AH]);
    v.Normalize();
    w = points[TORQUEROD_C] - points[TORQUEROD_AH];
    w.Normalize();
    u = Vcross(v, w);
    rot.SetFromDirectionAxes(u, v, w);

    m_torquerod[side] = chrono_types::make_shared<ChBody>();
    m_torquerod[side]->SetName(m_name + "_torquerod" + suffix);
    m_torquerod[side]->SetTag(m_obj_tag);
    m_torquerod[side]->SetPos(points[TORQUEROD_CM]);
    m_torquerod[side]->SetRot(rot);
    m_torquerod[side]->SetMass(getTorquerodMass());
    m_torquerod[side]->SetInertiaXX(getTorquerodInertia());
    chassis->GetSystem()->AddBody(m_torquerod[side]);

    // Create and initialize lower beam body.
    // Determine the rotation matrix of the lower link based on the plane of the hard points
    // (z-axis along the length of the lower link)
    v = Vcross(points[LOWERBEAM_C] - points[TORQUEROD_AH], points[LOWERBEAM_AH] - points[TORQUEROD_AH]);
    v.Normalize();
    w = points[LOWERBEAM_C] - points[LOWERBEAM_AH];
    w.Normalize();
    u = Vcross(v, w);
    rot.SetFromDirectionAxes(u, v, w);

    m_lowerbeam[side] = chrono_types::make_shared<ChBody>();
    m_lowerbeam[side]->SetName(m_name + "_lowerLink" + suffix);
    m_lowerbeam[side]->SetTag(m_obj_tag);
    m_lowerbeam[side]->SetPos(points[LOWERBEAM_CM]);
    m_lowerbeam[side]->SetRot(rot);
    m_lowerbeam[side]->SetMass(getLowerbeamMass());
    m_lowerbeam[side]->SetInertiaXX(getLowerbeamInertia());
    chassis->GetSystem()->AddBody(m_lowerbeam[side]);

    // Create and initialize the revolute joint between axle and knuckle.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction.
    w = points[KNUCKLE_U] - points[KNUCKLE_L];
    w.Normalize();
    u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
    u.Normalize();
    v = Vcross(w, u);
    rot.SetFromDirectionAxes(u, v, w);

    m_revoluteKingpin[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetName(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->Initialize(m_axlehousing, m_knuckle[side],
                                        ChFrame<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.GetQuaternion()));
    m_revoluteKingpin[side]->SetTag(m_obj_tag);
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the spherical joint between axle housing and torque rod.
    m_sphericalTorquerod[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalTorquerod[side]->SetName(m_name + "_sphericalTorquerod" + suffix);
    m_sphericalTorquerod[side]->SetTag(m_obj_tag);
    m_sphericalTorquerod[side]->Initialize(m_axlehousing, m_torquerod[side], ChFrame<>(points[TORQUEROD_AH], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTorquerod[side]);

    // Create and initialize the spherical joint between axle housing and lower beam.
    m_sphericalLowerbeam[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalLowerbeam[side]->SetName(m_name + "_sphericalLowerbeam" + suffix);
    m_sphericalLowerbeam[side]->SetTag(m_obj_tag);
    m_sphericalLowerbeam[side]->Initialize(m_axlehousing, m_lowerbeam[side], ChFrame<>(points[LOWERBEAM_AH], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLowerbeam[side]);

    // Create and initialize the revolute joint between chassis and torque rod.
    m_revoluteTorquerod[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteTorquerod[side]->SetName(m_name + "_revoluteTorquerod" + suffix);
    m_revoluteTorquerod[side]->SetTag(m_obj_tag);
    m_revoluteTorquerod[side]->Initialize(chassis->GetBody(), m_torquerod[side],
                                          ChFrame<>(points[TORQUEROD_C], chassisRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revoluteTorquerod[side]);

    // Create and initialize the revolute joint between chassis and lower beam.
    m_revoluteLowerbeam[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteLowerbeam[side]->SetName(m_name + "_revoluteLowerbeam" + suffix);
    m_revoluteLowerbeam[side]->SetTag(m_obj_tag);
    m_revoluteLowerbeam[side]->Initialize(chassis->GetBody(), m_lowerbeam[side],
                                          ChFrame<>(points[LOWERBEAM_C], chassisRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revoluteLowerbeam[side]);

    // Create and initialize the revolute joint between upright and spindle.
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetName(m_name + "_revolute" + suffix);
    m_revolute[side]->SetTag(m_obj_tag);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side],
                                 ChFrame<>(points[SPINDLE], chassisRot * QuatFromAngleX(CH_PI_2)));
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper between axle housing and chassis
    m_shockAH[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shockAH[side]->SetName(m_name + "_shockAH" + suffix);
    m_shockAH[side]->SetTag(m_obj_tag);
    m_shockAH[side]->Initialize(chassis->GetBody(), m_axlehousing, false, points[SHOCKAH_C], points[SHOCKAH_AH]);
    m_shockAH[side]->RegisterForceFunctor(getShockAHForceCallback());
    chassis->GetSystem()->AddLink(m_shockAH[side]);

    // Create and initialize the spring/damper between lower beam and chassis
    m_shockLB[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shockLB[side]->SetName(m_name + "_shockLB" + suffix);
    m_shockLB[side]->SetTag(m_obj_tag);
    m_shockLB[side]->Initialize(chassis->GetBody(), m_axlehousing, false, points[SHOCKLB_C], points[SHOCKLB_LB]);
    m_shockLB[side]->RegisterForceFunctor(getShockLBForceCallback());
    chassis->GetSystem()->AddLink(m_shockLB[side]);

    if (UseTierodBodies()) {
        // Orientation of tierod body
        w = (points[TIEROD_K] - points[TIEROD_C]).GetNormalized();
        u = chassisRot.GetAxisX();
        v = Vcross(w, u).GetNormalized();
        u = Vcross(v, w);
        rot.SetFromDirectionAxes(u, v, w);

        // Create the tierod body
        m_tierod[side] = chrono_types::make_shared<ChBody>();
        m_tierod[side]->SetName(m_name + "_tierodBody" + suffix);
        m_tierod[side]->SetTag(m_obj_tag);
        m_tierod[side]->SetPos((points[TIEROD_K] + points[TIEROD_C]) / 2);
        m_tierod[side]->SetRot(rot.GetQuaternion());
        m_tierod[side]->SetMass(getTierodMass());
        m_tierod[side]->SetInertiaXX(getTierodInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_tierod[side]);

        // Connect tierod body to knuckle (spherical) and chassis (universal)
        m_sphericalTierod[side] = chrono_types::make_shared<ChJoint>(
            ChJoint::Type::SPHERICAL, m_name + "_sphericalTierod" + suffix, m_knuckle[side], m_tierod[side],
            ChFrame<>(points[TIEROD_K], QUNIT), getTierodBushingData());
        m_sphericalTierod[side]->SetTag(m_obj_tag);
        chassis->AddJoint(m_sphericalTierod[side]);
        m_universalTierod[side] = chrono_types::make_shared<ChJoint>(
            ChJoint::Type::UNIVERSAL, m_name + "_universalTierod" + suffix, tierod_body, m_tierod[side],
            ChFrame<>(points[TIEROD_C], rot.GetQuaternion()), getTierodBushingData());
        m_universalTierod[side]->SetTag(m_obj_tag);
        chassis->AddJoint(m_universalTierod[side]);
    } else {
        // Create and initialize the tierod distance constraint between chassis and knuckle.
        m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
        m_distTierod[side]->SetName(m_name + "_distTierod" + suffix);
        m_distTierod[side]->SetTag(m_obj_tag);
        m_distTierod[side]->Initialize(tierod_body, m_knuckle[side], false, points[TIEROD_C], points[TIEROD_K]);
        chassis->GetSystem()->AddLink(m_distTierod[side]);
    }

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

void ChHendricksonPRIMAXX::InitializeInertiaProperties() {
    m_mass = getAxlehousingMass() + getTransversebeamMass() +
             2 * (getSpindleMass() + getKnuckleMass() + getTorquerodMass() + getLowerbeamMass());
    if (UseTierodBodies()) {
        m_mass += 2 * getTierodMass();
    }
}

void ChHendricksonPRIMAXX::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());
    ChMatrix33<> inertiaTorqueRod(getTorquerodInertia());
    ChMatrix33<> inertiaLowerbeam(getLowerbeamInertia());

    CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);

    composite.AddComponent(m_knuckle[LEFT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckle[RIGHT]->GetFrameCOMToAbs(), getKnuckleMass(), inertiaKnuckle);

    composite.AddComponent(m_torquerod[LEFT]->GetFrameCOMToAbs(), getTorquerodMass(), inertiaTorqueRod);
    composite.AddComponent(m_torquerod[RIGHT]->GetFrameCOMToAbs(), getTorquerodMass(), inertiaTorqueRod);

    composite.AddComponent(m_lowerbeam[LEFT]->GetFrameCOMToAbs(), getLowerbeamMass(), inertiaLowerbeam);
    composite.AddComponent(m_lowerbeam[RIGHT]->GetFrameCOMToAbs(), getLowerbeamMass(), inertiaLowerbeam);

    composite.AddComponent(m_axlehousing->GetFrameCOMToAbs(), getAxlehousingMass(),
                           ChMatrix33<>(getAxlehousingInertia()));
    composite.AddComponent(m_transversebeam->GetFrameCOMToAbs(), getTransversebeamMass(),
                           ChMatrix33<>(getTransversebeamInertia()));

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
double ChHendricksonPRIMAXX::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChHendricksonPRIMAXX::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("ShockLB", m_shockLB[side]->GetForce(), m_shockLB[side]->GetLength(),
                                        m_shockLB[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("ShockAH", m_shockAH[side]->GetForce(), m_shockAH[side]->GetLength(),
                                        m_shockAH[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogHardpointLocations(const ChVector3d& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector3d pos = ref + unit * getLocation(static_cast<PointId>(i));

        std::cout << "   " << m_pointNames[i] << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revolute[side]->GetConstraintViolation();
        std::cout << "Spindle revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revoluteKingpin[side]->GetConstraintViolation();
        std::cout << "Kingpin revolute      ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }
    // Spherical joints
    {
        ChVectorDynamic<> C = m_sphericalTorquerod[side]->GetConstraintViolation();
        std::cout << "Torquerod spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalLowerbeam[side]->GetConstraintViolation();
        std::cout << "Lowerbeam spherical          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "\n";
    }

    {
        ChVectorDynamic<> C = m_revoluteLowerbeam[side]->GetConstraintViolation();
        std::cout << "Lowerbeam revolute          ";
        std::cout << "  " << C(0) << "  ";
        std::cout << "  " << C(1) << "  ";
        std::cout << "  " << C(2) << "  ";
        std::cout << "  " << C(3) << "  ";
        std::cout << "  " << C(4) << "\n";
    }

    {
        ChVectorDynamic<> C = m_revoluteTorquerod[side]->GetConstraintViolation();
        std::cout << "Torquerod revolute          ";
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
void ChHendricksonPRIMAXX::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axlehousing, m_outerL, m_outerR, getAxlehousingRadius(), ChColor(1.0f, 0.0f, 0.0f));
    AddVisualizationLink(m_axlehousing, m_pointsL[LOWERBEAM_AH], m_pointsL[TORQUEROD_AH], getAxlehousingRadius() / 2,
                         ChColor(1.0f, 0.0f, 0.0f));
    AddVisualizationLink(m_axlehousing, m_pointsR[LOWERBEAM_AH], m_pointsR[TORQUEROD_AH], getAxlehousingRadius() / 2,
                         ChColor(1.0f, 0.0f, 0.0f));

    AddVisualizationLink(m_transversebeam, m_pointsL[LOWERBEAM_TB], m_pointsR[LOWERBEAM_TB], getTransversebeamRadius(),
                         ChColor(0.7f, 0.7f, 0.7f));

    AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius());
    AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius());

    AddVisualizationLink(m_torquerod[LEFT], m_pointsL[TORQUEROD_AH], m_pointsL[TORQUEROD_C], getTorquerodRadius(),
                         ChColor(0.6f, 0.2f, 0.6f));
    AddVisualizationLink(m_torquerod[RIGHT], m_pointsR[TORQUEROD_AH], m_pointsR[TORQUEROD_C], getTorquerodRadius(),
                         ChColor(0.6f, 0.2f, 0.6f));

    AddVisualizationLowerBeam(m_lowerbeam[LEFT], m_pointsL[LOWERBEAM_C], m_pointsL[LOWERBEAM_AH],
                              m_pointsL[LOWERBEAM_TB], getLowerbeamRadius(), ChColor(0.2f, 0.6f, 0.2f));
    AddVisualizationLowerBeam(m_lowerbeam[RIGHT], m_pointsR[LOWERBEAM_C], m_pointsR[LOWERBEAM_AH],
                              m_pointsR[LOWERBEAM_TB], getLowerbeamRadius(), ChColor(0.2f, 0.6f, 0.2f));

    // Add visualization for the springs and shocks
    m_shockLB[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shockLB[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    m_shockLB[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shockLB[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    m_shockAH[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shockAH[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    m_shockAH[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.06, 150, 15));
    m_shockAH[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());

    // Add visualization for the tie-rods
    if (UseTierodBodies()) {
        AddVisualizationTierod(m_tierod[LEFT], m_pointsL[TIEROD_C], m_pointsL[TIEROD_K], getTierodRadius());
        AddVisualizationTierod(m_tierod[RIGHT], m_pointsR[TIEROD_C], m_pointsR[TIEROD_K], getTierodRadius());
    } else {
        m_distTierod[LEFT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
        m_distTierod[RIGHT]->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
    }
}

void ChHendricksonPRIMAXX::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axlehousing);
    ChPart::RemoveVisualizationAssets(m_transversebeam);

    ChPart::RemoveVisualizationAssets(m_knuckle[LEFT]);
    ChPart::RemoveVisualizationAssets(m_knuckle[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_torquerod[LEFT]);
    ChPart::RemoveVisualizationAssets(m_torquerod[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_lowerbeam[LEFT]);
    ChPart::RemoveVisualizationAssets(m_lowerbeam[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shockLB[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shockLB[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shockAH[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shockAH[RIGHT]);

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
//

void ChHendricksonPRIMAXX::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                const ChVector3d pt_1,
                                                const ChVector3d pt_2,
                                                double radius,
                                                const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector3d p_2 = body->TransformPointParentToLocal(pt_2);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
}

void ChHendricksonPRIMAXX::AddVisualizationLowerBeam(std::shared_ptr<ChBody> body,
                                                     const ChVector3d pt_C,
                                                     const ChVector3d pt_AH,
                                                     const ChVector3d pt_TB,
                                                     double radius,
                                                     const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector3d p_C = body->TransformPointParentToLocal(pt_C);
    ChVector3d p_AH = body->TransformPointParentToLocal(pt_AH);
    ChVector3d p_TB = body->TransformPointParentToLocal(pt_TB);

    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_C, p_AH, radius);
    utils::ChBodyGeometry::AddVisualizationCylinder(body, p_AH, p_TB, radius);
}

void ChHendricksonPRIMAXX::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                                   const ChVector3d pt_U,
                                                   const ChVector3d pt_L,
                                                   const ChVector3d pt_T,
                                                   double radius)

{
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector3d p_U = knuckle->TransformPointParentToLocal(pt_U);
    ChVector3d p_L = knuckle->TransformPointParentToLocal(pt_L);
    ChVector3d p_T = knuckle->TransformPointParentToLocal(pt_T);

    if (p_L.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_L, VNULL, radius);
    }

    if (p_U.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_U, VNULL, radius);
    }

    if (p_T.Length2() > threshold2) {
        utils::ChBodyGeometry::AddVisualizationCylinder(knuckle, p_T, VNULL, radius);
    }
}

void ChHendricksonPRIMAXX::AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
                                                  const ChVector3d pt_C,
                                                  const ChVector3d pt_U,
                                                  double radius) {
    // Express hardpoint locations in body frame.
    ChVector3d p_C = tierod->TransformPointParentToLocal(pt_C);
    ChVector3d p_U = tierod->TransformPointParentToLocal(pt_U);

    utils::ChBodyGeometry::AddVisualizationCylinder(tierod, p_C, p_U, radius);
}

// -----------------------------------------------------------------------------

void ChHendricksonPRIMAXX::PopulateComponentList() {
    m_bodies.push_back(m_spindle[0]);
    m_bodies.push_back(m_spindle[1]);
    m_bodies.push_back(m_knuckle[0]);
    m_bodies.push_back(m_knuckle[1]);
    m_bodies.push_back(m_torquerod[0]);
    m_bodies.push_back(m_torquerod[1]);
    m_bodies.push_back(m_lowerbeam[0]);
    m_bodies.push_back(m_lowerbeam[1]);
    m_bodies.push_back(m_transversebeam);
    m_bodies.push_back(m_axlehousing);
    if (UseTierodBodies()) {
        m_bodies.push_back(m_tierod[0]);
        m_bodies.push_back(m_tierod[1]);
    }

    m_shafts.push_back(m_axle[0]);
    m_shafts.push_back(m_axle[1]);

    m_joints.push_back(m_revolute[0]);
    m_joints.push_back(m_revolute[1]);
    m_joints.push_back(m_revoluteKingpin[0]);
    m_joints.push_back(m_revoluteKingpin[1]);
    m_joints.push_back(m_sphericalTorquerod[0]);
    m_joints.push_back(m_sphericalTorquerod[1]);
    m_joints.push_back(m_revoluteTorquerod[0]);
    m_joints.push_back(m_revoluteTorquerod[1]);
    m_joints.push_back(m_sphericalLowerbeam[0]);
    m_joints.push_back(m_sphericalLowerbeam[1]);
    m_joints.push_back(m_revoluteLowerbeam[0]);
    m_joints.push_back(m_revoluteLowerbeam[1]);
    m_joints.push_back(m_sphericalTB[0]);
    m_joints.push_back(m_sphericalTB[1]);
    if (UseTierodBodies()) {
        m_sphericalTierod[0]->IsKinematic() ? m_joints.push_back(m_sphericalTierod[0]->GetAsLink())
                                            : m_body_loads.push_back(m_sphericalTierod[0]->GetAsBushing());
        m_sphericalTierod[1]->IsKinematic() ? m_joints.push_back(m_sphericalTierod[1]->GetAsLink())
                                            : m_body_loads.push_back(m_sphericalTierod[1]->GetAsBushing());
        m_universalTierod[0]->IsKinematic() ? m_joints.push_back(m_universalTierod[0]->GetAsLink())
                                            : m_body_loads.push_back(m_universalTierod[0]->GetAsBushing());
        m_universalTierod[1]->IsKinematic() ? m_joints.push_back(m_universalTierod[1]->GetAsLink())
                                            : m_body_loads.push_back(m_universalTierod[1]->GetAsBushing());
    } else {
        m_joints.push_back(m_distTierod[0]);
        m_joints.push_back(m_distTierod[1]);
    }

    m_tsdas.push_back(m_shockLB[0]);
    m_tsdas.push_back(m_shockLB[1]);
    m_tsdas.push_back(m_shockAH[0]);
    m_tsdas.push_back(m_shockAH[1]);
}

}  // end namespace vehicle
}  // end namespace chrono
