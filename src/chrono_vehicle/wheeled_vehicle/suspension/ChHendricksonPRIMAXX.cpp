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

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChPointPointShape.h"

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
    auto sys = m_transversebeam->GetSystem();
    if (sys) {
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
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::Initialize(std::shared_ptr<ChChassis> chassis,
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

    // Transform the location of the axle body COM to absolute frame.
    ChVector<> axleCOM_local = getAxlehousingCOM();
    ChVector<> axleCOM = suspension_to_abs.TransformLocalToParent(axleCOM_local);

    // Calculate end points on the axle body, expressed in the absolute frame
    // (for visualization)
    ChVector<> midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
    ChVector<> outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    m_outerL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_outerR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    // Create and initialize the axle housing body.
    m_axlehousing = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
    m_axlehousing->SetNameString(m_name + "_axlehousing");
    m_axlehousing->SetPos(axleCOM);
    m_axlehousing->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_axlehousing->SetMass(getAxlehousingMass());
    m_axlehousing->SetInertiaXX(getAxlehousingInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axlehousing);

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

    // Create transverse beam body
    ChVector<> tbCOM_local = getTransversebeamCOM();
    ChVector<> tbCOM = suspension_to_abs.TransformLocalToParent(tbCOM_local);

    m_transversebeam = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
    m_transversebeam->SetNameString(m_name + "_transversebeam");
    m_transversebeam->SetPos(tbCOM);
    m_transversebeam->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
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

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_knuckle[side]->SetNameString(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(chassisRot);
    m_knuckle[side]->SetMass(getKnuckleMass());
    m_knuckle[side]->SetInertiaXX(getKnuckleInertia());
    chassis->GetSystem()->AddBody(m_knuckle[side]);

    // Create and initialize spindle body (same orientation as the chassis)
    m_spindle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_spindle[side]->SetNameString(m_name + "_spindle" + suffix);
    m_spindle[side]->SetPos(points[SPINDLE]);
    m_spindle[side]->SetRot(chassisRot);
    m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
    m_spindle[side]->SetMass(getSpindleMass());
    m_spindle[side]->SetInertiaXX(getSpindleInertia());
    chassis->GetSystem()->AddBody(m_spindle[side]);

    // Create and initialize torque rod body.
    // Determine the rotation matrix of the torque rod based on the plane of the hard points
    // (z-axis along the length of the torque rod)
    v = Vcross(points[TORQUEROD_AH] - points[LOWERBEAM_AH], points[TORQUEROD_C] - points[LOWERBEAM_AH]);
    v.Normalize();
    w = points[TORQUEROD_C] - points[TORQUEROD_AH];
    w.Normalize();
    u = Vcross(v, w);
    rot.Set_A_axis(u, v, w);

    m_torquerod[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_torquerod[side]->SetNameString(m_name + "_torquerod" + suffix);
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
    rot.Set_A_axis(u, v, w);

    m_lowerbeam[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_lowerbeam[side]->SetNameString(m_name + "_lowerLink" + suffix);
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
    rot.Set_A_axis(u, v, w);

    m_revoluteKingpin[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->Initialize(
        m_axlehousing, m_knuckle[side],
        ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the spherical joint between axle housing and torque rod.
    m_sphericalTorquerod[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalTorquerod[side]->SetNameString(m_name + "_sphericalTorquerod" + suffix);
    m_sphericalTorquerod[side]->Initialize(m_axlehousing, m_torquerod[side], ChCoordsys<>(points[TORQUEROD_AH], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalTorquerod[side]);

    // Create and initialize the spherical joint between axle housing and lower beam.
    m_sphericalLowerbeam[side] = chrono_types::make_shared<ChLinkLockSpherical>();
    m_sphericalLowerbeam[side]->SetNameString(m_name + "_sphericalLowerbeam" + suffix);
    m_sphericalLowerbeam[side]->Initialize(m_axlehousing, m_lowerbeam[side], ChCoordsys<>(points[LOWERBEAM_AH], QUNIT));
    chassis->GetSystem()->AddLink(m_sphericalLowerbeam[side]);

    // Create and initialize the revolute joint between chassis and torque rod.
    ChCoordsys<> revTR_csys(points[TORQUEROD_C], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revoluteTorquerod[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteTorquerod[side]->SetNameString(m_name + "_revoluteTorquerod" + suffix);
    m_revoluteTorquerod[side]->Initialize(chassis->GetBody(), m_torquerod[side], revTR_csys);
    chassis->GetSystem()->AddLink(m_revoluteTorquerod[side]);

    // Create and initialize the revolute joint between chassis and lower beam.
    ChCoordsys<> revLB_csys(points[LOWERBEAM_C], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revoluteLowerbeam[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteLowerbeam[side]->SetNameString(m_name + "_revoluteLowerbeam" + suffix);
    m_revoluteLowerbeam[side]->Initialize(chassis->GetBody(), m_lowerbeam[side], revLB_csys);
    chassis->GetSystem()->AddLink(m_revoluteLowerbeam[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));

    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper between axle housing and chassis
    m_shockAH[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shockAH[side]->SetNameString(m_name + "_shockAH" + suffix);
    m_shockAH[side]->Initialize(chassis->GetBody(), m_axlehousing, false, points[SHOCKAH_C], points[SHOCKAH_AH]);
    m_shockAH[side]->RegisterForceFunctor(getShockAHForceCallback());
    chassis->GetSystem()->AddLink(m_shockAH[side]);

    // Create and initialize the spring/damper between lower beam and chassis
    m_shockLB[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shockLB[side]->SetNameString(m_name + "_shockLB" + suffix);
    m_shockLB[side]->Initialize(chassis->GetBody(), m_axlehousing, false, points[SHOCKLB_C], points[SHOCKLB_LB]);
    m_shockLB[side]->RegisterForceFunctor(getShockLBForceCallback());
    chassis->GetSystem()->AddLink(m_shockLB[side]);

    if (UseTierodBodies()) {
        // Orientation of tierod body
        w = (points[TIEROD_K] - points[TIEROD_C]).GetNormalized();
        u = chassisRot.GetXaxis();
        v = Vcross(w, u).GetNormalized();
        u = Vcross(v, w);
        rot.Set_A_axis(u, v, w);

        // Create the tierod body
        m_tierod[side] = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
        m_tierod[side]->SetNameString(m_name + "_tierodBody" + suffix);
        m_tierod[side]->SetPos((points[TIEROD_K] + points[TIEROD_C]) / 2);
        m_tierod[side]->SetRot(rot.Get_A_quaternion());
        m_tierod[side]->SetMass(getTierodMass());
        m_tierod[side]->SetInertiaXX(getTierodInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_tierod[side]);

        // Connect tierod body to knuckle (spherical) and chassis (universal)
        m_sphericalTierod[side] = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::SPHERICAL, m_name + "_sphericalTierod" + suffix, m_knuckle[side], m_tierod[side],
            ChCoordsys<>(points[TIEROD_K], QUNIT), getTierodBushingData());
        chassis->AddJoint(m_sphericalTierod[side]);
        m_universalTierod[side] = chrono_types::make_shared<ChVehicleJoint>(
            ChVehicleJoint::Type::UNIVERSAL, m_name + "_universalTierod" + suffix, tierod_body, m_tierod[side],
            ChCoordsys<>(points[TIEROD_C], rot.Get_A_quaternion()), getTierodBushingData());
        chassis->AddJoint(m_universalTierod[side]);
    } else {
        // Create and initialize the tierod distance constraint between chassis and knuckle.
        m_distTierod[side] = chrono_types::make_shared<ChLinkDistance>();
        m_distTierod[side]->SetNameString(m_name + "_distTierod" + suffix);
        m_distTierod[side]->Initialize(tierod_body, m_knuckle[side], false, points[TIEROD_C], points[TIEROD_K]);
        chassis->GetSystem()->AddLink(m_distTierod[side]);
    }

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

void ChHendricksonPRIMAXX::InitializeInertiaProperties() {
    m_mass = getAxlehousingMass() + getTransversebeamMass() +
             2 * (getSpindleMass() + getKnuckleMass() + getTorquerodMass() + getLowerbeamMass());
    if (UseTierodBodies()) {
        m_mass += 2 * getTierodMass();
    }
}

void ChHendricksonPRIMAXX::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());
    ChMatrix33<> inertiaTorqueRod(getTorquerodInertia());
    ChMatrix33<> inertiaLowerbeam(getLowerbeamInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);

    composite.AddComponent(m_knuckle[LEFT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckle[RIGHT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);

    composite.AddComponent(m_torquerod[LEFT]->GetFrame_COG_to_abs(), getTorquerodMass(), inertiaTorqueRod);
    composite.AddComponent(m_torquerod[RIGHT]->GetFrame_COG_to_abs(), getTorquerodMass(), inertiaTorqueRod);

    composite.AddComponent(m_lowerbeam[LEFT]->GetFrame_COG_to_abs(), getLowerbeamMass(), inertiaLowerbeam);
    composite.AddComponent(m_lowerbeam[RIGHT]->GetFrame_COG_to_abs(), getLowerbeamMass(), inertiaLowerbeam);

    composite.AddComponent(m_axlehousing->GetFrame_COG_to_abs(), getAxlehousingMass(), ChMatrix33<>(getAxlehousingInertia()));
    composite.AddComponent(m_transversebeam->GetFrame_COG_to_abs(), getTransversebeamMass(),
                           ChMatrix33<>(getTransversebeamInertia()));

    if (UseTierodBodies()) {
        ChMatrix33<> inertiaTierod(getTierodInertia());
        composite.AddComponent(m_tierod[LEFT]->GetFrame_COG_to_abs(), getTierodMass(), inertiaTierod);
        composite.AddComponent(m_tierod[RIGHT]->GetFrame_COG_to_abs(), getTierodMass(), inertiaTierod);
    }

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
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
ChSuspension::Force ChHendricksonPRIMAXX::ReportSuspensionForce(VehicleSide side) const {
    ChSuspension::Force force;

    force.spring_force = m_shockLB[side]->GetForce();
    force.spring_length = m_shockLB[side]->GetLength();
    force.spring_velocity = m_shockLB[side]->GetVelocity();

    force.shock_force = m_shockAH[side]->GetForce();
    force.shock_length = m_shockAH[side]->GetLength();
    force.shock_velocity = m_shockAH[side]->GetVelocity();

    return force;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChHendricksonPRIMAXX::LogConstraintViolations(VehicleSide side) {
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revolute[side]->GetConstraintViolation();
        GetLog() << "Spindle revolute      ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
    {
        ChVectorDynamic<> C = m_revoluteKingpin[side]->GetConstraintViolation();
        GetLog() << "Kingpin revolute      ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }
    // Spherical joints
    {
        ChVectorDynamic<> C = m_sphericalTorquerod[side]->GetConstraintViolation();
        GetLog() << "Torquerod spherical          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalLowerbeam[side]->GetConstraintViolation();
        GetLog() << "Lowerbeam spherical          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }

    {
        ChVectorDynamic<> C = m_revoluteLowerbeam[side]->GetConstraintViolation();
        GetLog() << "Lowerbeam revolute          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }

    {
        ChVectorDynamic<> C = m_revoluteTorquerod[side]->GetConstraintViolation();
        GetLog() << "Torquerod revolute          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
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
    m_shockLB[LEFT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shockLB[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());

    m_shockLB[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shockLB[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());

    m_shockAH[LEFT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shockAH[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());

    m_shockAH[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shockAH[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());


    // Add visualization for the tie-rods
    if (UseTierodBodies()) {
        AddVisualizationTierod(m_tierod[LEFT], m_pointsL[TIEROD_C], m_pointsL[TIEROD_K], getTierodRadius());
        AddVisualizationTierod(m_tierod[RIGHT], m_pointsR[TIEROD_C], m_pointsR[TIEROD_K], getTierodRadius());
    } else {
        m_distTierod[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
        m_distTierod[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
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
                                                const ChVector<> pt_1,
                                                const ChVector<> pt_2,
                                                double radius,
                                                const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = p_1;
    cyl->GetCylinderGeometry().p2 = p_2;
    cyl->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl);
}

void ChHendricksonPRIMAXX::AddVisualizationLowerBeam(std::shared_ptr<ChBody> body,
                                                     const ChVector<> pt_C,
                                                     const ChVector<> pt_AH,
                                                     const ChVector<> pt_TB,
                                                     double radius,
                                                     const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector<> p_C = body->TransformPointParentToLocal(pt_C);
    ChVector<> p_AH = body->TransformPointParentToLocal(pt_AH);
    ChVector<> p_TB = body->TransformPointParentToLocal(pt_TB);

    auto cyl_1 = chrono_types::make_shared<ChCylinderShape>();
    cyl_1->GetCylinderGeometry().p1 = p_C;
    cyl_1->GetCylinderGeometry().p2 = p_AH;
    cyl_1->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl_1);

    auto cyl_2 = chrono_types::make_shared<ChCylinderShape>();
    cyl_2->GetCylinderGeometry().p1 = p_AH;
    cyl_2->GetCylinderGeometry().p2 = p_TB;
    cyl_2->GetCylinderGeometry().rad = radius;
    body->AddVisualShape(cyl_2);
}

void ChHendricksonPRIMAXX::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                                   const ChVector<> pt_U,
                                                   const ChVector<> pt_L,
                                                   const ChVector<> pt_T,
                                                   double radius)

{
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_U = knuckle->TransformPointParentToLocal(pt_U);
    ChVector<> p_L = knuckle->TransformPointParentToLocal(pt_L);
    ChVector<> p_T = knuckle->TransformPointParentToLocal(pt_T);

    if (p_L.Length2() > threshold2) {
        auto cyl_L = chrono_types::make_shared<ChCylinderShape>();
        cyl_L->GetCylinderGeometry().p1 = p_L;
        cyl_L->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_L->GetCylinderGeometry().rad = radius;
        knuckle->AddVisualShape(cyl_L);
    }

    if (p_U.Length2() > threshold2) {
        auto cyl_U = chrono_types::make_shared<ChCylinderShape>();
        cyl_U->GetCylinderGeometry().p1 = p_U;
        cyl_U->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_U->GetCylinderGeometry().rad = radius;
        knuckle->AddVisualShape(cyl_U);
    }

    if (p_T.Length2() > threshold2) {
        auto cyl_T = chrono_types::make_shared<ChCylinderShape>();
        cyl_T->GetCylinderGeometry().p1 = p_T;
        cyl_T->GetCylinderGeometry().p2 = ChVector<>(0, 0, 0);
        cyl_T->GetCylinderGeometry().rad = radius;
        knuckle->AddVisualShape(cyl_T);
    }
}

void ChHendricksonPRIMAXX::AddVisualizationTierod(std::shared_ptr<ChBody> tierod,
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
void ChHendricksonPRIMAXX::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    bodies.push_back(m_torquerod[0]);
    bodies.push_back(m_torquerod[1]);
    bodies.push_back(m_lowerbeam[0]);
    bodies.push_back(m_lowerbeam[1]);
    bodies.push_back(m_transversebeam);
    bodies.push_back(m_axlehousing);
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
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    joints.push_back(m_sphericalTorquerod[0]);
    joints.push_back(m_sphericalTorquerod[1]);
    joints.push_back(m_revoluteTorquerod[0]);
    joints.push_back(m_revoluteTorquerod[1]);
    joints.push_back(m_sphericalLowerbeam[0]);
    joints.push_back(m_sphericalLowerbeam[1]);
    joints.push_back(m_revoluteLowerbeam[0]);
    joints.push_back(m_revoluteLowerbeam[1]);
    joints.push_back(m_sphericalTB[0]);
    joints.push_back(m_sphericalTB[1]);
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
    springs.push_back(m_shockLB[0]);
    springs.push_back(m_shockLB[1]);
    springs.push_back(m_shockAH[0]);
    springs.push_back(m_shockAH[1]);
    ChPart::ExportLinSpringList(jsonDocument, springs);
}

void ChHendricksonPRIMAXX::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    bodies.push_back(m_torquerod[0]);
    bodies.push_back(m_torquerod[1]);
    bodies.push_back(m_lowerbeam[0]);
    bodies.push_back(m_lowerbeam[1]);
    bodies.push_back(m_transversebeam);
    bodies.push_back(m_axlehousing);
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
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    joints.push_back(m_sphericalTorquerod[0]);
    joints.push_back(m_sphericalTorquerod[1]);
    joints.push_back(m_revoluteTorquerod[0]);
    joints.push_back(m_revoluteTorquerod[1]);
    joints.push_back(m_sphericalLowerbeam[0]);
    joints.push_back(m_sphericalLowerbeam[1]);
    joints.push_back(m_revoluteLowerbeam[0]);
    joints.push_back(m_revoluteLowerbeam[1]);
    joints.push_back(m_sphericalTB[0]);
    joints.push_back(m_sphericalTB[1]);
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
    springs.push_back(m_shockLB[0]);
    springs.push_back(m_shockLB[1]);
    springs.push_back(m_shockAH[0]);
    springs.push_back(m_shockAH[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
