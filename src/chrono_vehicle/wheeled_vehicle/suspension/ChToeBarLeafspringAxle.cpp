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
// Authors: Rainer Gericke, Radu Serban
// =============================================================================
//
// Base class for a steerable leaf-spring solid axle suspension.
//
// This class is meant for modelling a very simple steerable solid leafspring
// axle. The guiding function of leafspring is modelled by a ChLinkLockRevolutePrismatic
// joint, it allows vertical movement and tilting of the axle tube but no elasticity.
// The spring function of the leafspring is modelled by a vertical spring element.
// Tie up of the leafspring is not possible with this approach, as well as the
// characteristic kinematics along wheel travel. The roll center and roll stability
// is met well, if spring track is defined correctly. The class has been designed
// for maximum easyness and numerical efficiency.
//
// This axle subsystem works with the ChRotaryArm steering subsystem.
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
#include "chrono/assets/ChPointPointShape.h"

#include "chrono_vehicle/wheeled_vehicle/suspension/ChToeBarLeafspringAxle.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------
const std::string ChToeBarLeafspringAxle::m_pointNames[] = {"SHOCK_A    ", "SHOCK_C    ", "KNUCKLE_L  ", "KNUCKLE_U  ",
                                                            "KNUCKLE_DRL", "SPRING_A   ", "SPRING_C   ", "TIEROD_C   ",
                                                            "TIEROD_K   ", "SPINDLE    ", "KNUCKLE_CM "};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
ChToeBarLeafspringAxle::ChToeBarLeafspringAxle(const std::string& name) : ChSuspension(name) {}

ChToeBarLeafspringAxle::~ChToeBarLeafspringAxle() {
    auto sys = m_axleTube->GetSystem();
    if (sys) {
        sys->Remove(m_axleTube);
        sys->Remove(m_tierod);
        sys->Remove(m_draglink);
        sys->Remove(m_axleTubeGuide);
        sys->Remove(m_sphericalTierod);
        sys->Remove(m_sphericalDraglink);
        sys->Remove(m_universalDraglink);
        sys->Remove(m_universalTierod);

        for (int i = 0; i < 2; i++) {
            sys->Remove(m_knuckle[i]);
            sys->Remove(m_revoluteKingpin[i]);
            sys->Remove(m_shock[i]);
            sys->Remove(m_spring[i]);
        }
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarLeafspringAxle::Initialize(std::shared_ptr<ChChassis> chassis,
                                        std::shared_ptr<ChSubchassis> subchassis,
                                        std::shared_ptr<ChSteering> steering,
                                        const ChVector<>& location,
                                        double left_ang_vel,
                                        double right_ang_vel) {
    ChSuspension::Initialize(chassis, subchassis, steering, location, left_ang_vel, right_ang_vel);

    m_parent = chassis;
    m_rel_loc = location;

    m_left_knuckle_steers = isLeftKnuckleActuated();

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Express the suspension reference frame in the absolute coordinate system.
    ChFrame<> suspension_to_abs(location);
    suspension_to_abs.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Transform the location of the axle body COM to absolute frame.
    ChVector<> axleCOM_local = getAxleTubeCOM();
    ChVector<> axleCOM = suspension_to_abs.TransformLocalToParent(axleCOM_local);

    // Calculate end points on the axle body, expressed in the absolute frame
    // (for visualization)
    ChVector<> midpoint_local = 0.5 * (getLocation(KNUCKLE_U) + getLocation(KNUCKLE_L));
    ChVector<> outer_local(axleCOM_local.x(), midpoint_local.y(), axleCOM_local.z());
    m_axleOuterL = suspension_to_abs.TransformPointLocalToParent(outer_local);
    outer_local.y() = -outer_local.y();
    m_axleOuterR = suspension_to_abs.TransformPointLocalToParent(outer_local);

    // Create and initialize the axle body.
    m_axleTube = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
    m_axleTube->SetNameString(m_name + "_axleTube");
    m_axleTube->SetPos(axleCOM);
    m_axleTube->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_axleTube->SetMass(getAxleTubeMass());
    m_axleTube->SetInertiaXX(getAxleTubeInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_axleTube);

    // Fix the axle body to the chassis
    m_axleTubeGuide = chrono_types::make_shared<ChLinkLockRevolutePrismatic>();
    m_axleTubeGuide->SetNameString(m_name + "_revolutePrismaticAxleTube");
    const ChQuaternion<>& guideRot = chassis->GetBody()->GetFrame_REF_to_abs().GetRot();
    m_axleTubeGuide->Initialize(chassis->GetBody(), m_axleTube,
                                ChCoordsys<>(axleCOM, guideRot * Q_from_AngY(CH_C_PI_2)));
    chassis->GetBody()->GetSystem()->AddLink(m_axleTubeGuide);

    // Calculate end points on the tierod body, expressed in the absolute frame
    // (for visualization)
    ChVector<> tierodOuter_local(getLocation(TIEROD_K));
    m_tierodOuterL = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);
    tierodOuter_local.y() = -tierodOuter_local.y();
    m_tierodOuterR = suspension_to_abs.TransformPointLocalToParent(tierodOuter_local);

    // Create and initialize the tierod body.
    m_tierod = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
    m_tierod->SetNameString(m_name + "_tierodBody");
    m_tierod->SetPos((m_tierodOuterL + m_tierodOuterR) / 2);
    m_tierod->SetRot(chassis->GetBody()->GetFrame_REF_to_abs().GetRot());
    m_tierod->SetMass(getTierodMass());
    m_tierod->SetInertiaXX(getTierodInertia());
    chassis->GetBody()->GetSystem()->AddBody(m_tierod);

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
    InitializeSide(LEFT, chassis->GetBody(), m_pointsL, left_ang_vel);
    InitializeSide(RIGHT, chassis->GetBody(), m_pointsR, right_ang_vel);

    // Create connection to steering mechanism
    std::shared_ptr<ChBody> tierod_body = (steering == nullptr) ? chassis->GetBody() : steering->GetSteeringLink();

    if (m_left_knuckle_steers) {
        // Create and initialize the draglink body (one side only).
        // Determine the rotation matrix of the draglink based on the plane of the hard points
        // (z-axis along the length of the draglink)
        v = Vcross(m_pointsL[KNUCKLE_DRL], m_pointsL[DRAGLINK_C]);
        v.Normalize();
        w = m_pointsL[DRAGLINK_C] - m_pointsL[KNUCKLE_DRL];
        w.Normalize();
        u = Vcross(v, w);
        rot.Set_A_axis(u, v, w);

        m_draglink = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
        m_draglink->SetNameString(m_name + "_draglink");
        m_draglink->SetPos((m_pointsL[DRAGLINK_C] + m_pointsL[KNUCKLE_DRL]) / 2);
        m_draglink->SetRot(rot.Get_A_quaternion());
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetNameString(m_name + "_sphericalDraglink" + "_L");
        m_sphericalDraglink->Initialize(m_draglink, tierod_body, ChCoordsys<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize the universal joint between draglink and knuckle
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetNameString(m_name + "_universalDraglink" + "_L");
        m_universalDraglink->Initialize(m_draglink, m_knuckle[LEFT],
                                        ChFrame<>(m_pointsL[KNUCKLE_DRL], rot.Get_A_quaternion()));
        chassis->GetBody()->GetSystem()->AddLink(m_universalDraglink);
    } else {
        // Create and initialize the draglink body (one side only).
        // Determine the rotation matrix of the draglink based on the plane of the hard points
        // (z-axis along the length of the draglink)
        v = Vcross(m_pointsR[KNUCKLE_DRL], m_pointsL[DRAGLINK_C]);
        v.Normalize();
        w = m_pointsL[DRAGLINK_C] - m_pointsR[KNUCKLE_DRL];
        w.Normalize();
        u = Vcross(v, w);
        rot.Set_A_axis(u, v, w);

        m_draglink = std::shared_ptr<ChBody>(chassis->GetBody()->GetSystem()->NewBody());
        m_draglink->SetNameString(m_name + "_draglink");
        m_draglink->SetPos((m_pointsL[DRAGLINK_C] + m_pointsR[KNUCKLE_DRL]) / 2);
        m_draglink->SetRot(rot.Get_A_quaternion());
        m_draglink->SetMass(getDraglinkMass());
        m_draglink->SetInertiaXX(getDraglinkInertia());
        chassis->GetBody()->GetSystem()->AddBody(m_draglink);

        // Create and initialize the spherical joint between steering mechanism and draglink.
        m_sphericalDraglink = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalDraglink->SetNameString(m_name + "_sphericalDraglink" + "_L");
        m_sphericalDraglink->Initialize(m_draglink, tierod_body, ChCoordsys<>(m_pointsL[DRAGLINK_C], QUNIT));
        chassis->GetBody()->GetSystem()->AddLink(m_sphericalDraglink);

        // Create and initialize the universal joint between draglink and knuckle
        m_universalDraglink = chrono_types::make_shared<ChLinkUniversal>();
        m_universalDraglink->SetNameString(m_name + "_universalDraglink" + "_R");
        m_universalDraglink->Initialize(m_draglink, m_knuckle[RIGHT],
                                        ChFrame<>(m_pointsR[KNUCKLE_DRL], rot.Get_A_quaternion()));
        chassis->GetBody()->GetSystem()->AddLink(m_universalDraglink);
    }
}

void ChToeBarLeafspringAxle::InitializeSide(VehicleSide side,
                                            std::shared_ptr<ChBodyAuxRef> chassis,
                                            const std::vector<ChVector<>>& points,
                                            double ang_vel) {
    std::string suffix = (side == LEFT) ? "_L" : "_R";

    // Unit vectors for orientation matrices.
    ChVector<> u;
    ChVector<> v;
    ChVector<> w;
    ChMatrix33<> rot;

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetFrame_REF_to_abs().GetRot();

    // Spindle orientation (based on camber and toe angles)
    double sign = (side == LEFT) ? -1 : +1;
    auto spindleRot = chassisRot * Q_from_AngZ(sign * getToeAngle()) * Q_from_AngX(sign * getCamberAngle());

    // Create and initialize knuckle body (same orientation as the chassis)
    m_knuckle[side] = std::shared_ptr<ChBody>(chassis->GetSystem()->NewBody());
    m_knuckle[side]->SetNameString(m_name + "_knuckle" + suffix);
    m_knuckle[side]->SetPos(points[KNUCKLE_CM]);
    m_knuckle[side]->SetRot(spindleRot);
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

    // Create and initialize the joint between knuckle and tierod (one side has universal, the other has spherical).
    if (side == LEFT) {
        m_sphericalTierod = chrono_types::make_shared<ChLinkLockSpherical>();
        m_sphericalTierod->SetNameString(m_name + "_sphericalTierod" + suffix);
        m_sphericalTierod->Initialize(m_tierod, m_knuckle[side], ChCoordsys<>(points[TIEROD_K], QUNIT));
        chassis->GetSystem()->AddLink(m_sphericalTierod);
    } else {
        m_universalTierod = chrono_types::make_shared<ChLinkUniversal>();
        m_universalTierod->SetNameString(m_name + "_universalTierod" + suffix);
        m_universalTierod->Initialize(m_tierod, m_knuckle[side],
                                      ChFrame<>(points[TIEROD_K], chassisRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X)));
        chassis->GetSystem()->AddLink(m_universalTierod);
    }

    // Create and initialize the revolute joint between axle and knuckle.
    // Determine the joint orientation matrix from the hardpoint locations by
    // constructing a rotation matrix with the z axis along the joint direction.
    w = points[KNUCKLE_L] - points[KNUCKLE_U];
    w.Normalize();
    u = Vcross(points[KNUCKLE_U] - points[SPINDLE], points[KNUCKLE_L] - points[SPINDLE]);
    u.Normalize();
    v = Vcross(w, u);
    rot.Set_A_axis(u, v, w);

    m_revoluteKingpin[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revoluteKingpin[side]->SetNameString(m_name + "_revoluteKingpin" + suffix);
    m_revoluteKingpin[side]->Initialize(
        m_axleTube, m_knuckle[side], ChCoordsys<>((points[KNUCKLE_U] + points[KNUCKLE_L]) / 2, rot.Get_A_quaternion()));
    chassis->GetSystem()->AddLink(m_revoluteKingpin[side]);

    // Create and initialize the revolute joint between upright and spindle.
    ChCoordsys<> rev_csys(points[SPINDLE], spindleRot * Q_from_AngAxis(CH_C_PI / 2.0, VECT_X));
    m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
    m_revolute[side]->SetNameString(m_name + "_revolute" + suffix);
    m_revolute[side]->Initialize(m_spindle[side], m_knuckle[side], rev_csys);
    chassis->GetSystem()->AddLink(m_revolute[side]);

    // Create and initialize the spring/damper
    m_shock[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_shock[side]->SetNameString(m_name + "_shock" + suffix);
    m_shock[side]->Initialize(chassis, m_axleTube, false, points[SHOCK_C], points[SHOCK_A]);
    m_shock[side]->SetRestLength(getShockRestLength());
    m_shock[side]->RegisterForceFunctor(getShockForceFunctor());
    chassis->GetSystem()->AddLink(m_shock[side]);

    m_spring[side] = chrono_types::make_shared<ChLinkTSDA>();
    m_spring[side]->SetNameString(m_name + "_spring" + suffix);
    m_spring[side]->Initialize(chassis, m_axleTube, false, points[SPRING_C], points[SPRING_A]);
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

void ChToeBarLeafspringAxle::InitializeInertiaProperties() {
    m_mass = getAxleTubeMass() + getTierodMass() + getDraglinkMass() + 2 * (getSpindleMass() + getKnuckleMass());
}

void ChToeBarLeafspringAxle::UpdateInertiaProperties() {
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    ChMatrix33<> inertiaKnuckle(getKnuckleInertia());

    utils::CompositeInertia composite;
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_axleTube->GetFrame_COG_to_abs(), getAxleTubeMass(), ChMatrix33<>(getAxleTubeInertia()));
    composite.AddComponent(m_tierod->GetFrame_COG_to_abs(), getTierodMass(), ChMatrix33<>(getTierodInertia()));
    composite.AddComponent(m_draglink->GetFrame_COG_to_abs(), getDraglinkMass(), ChMatrix33<>(getDraglinkInertia()));
    composite.AddComponent(m_knuckle[LEFT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);
    composite.AddComponent(m_knuckle[RIGHT]->GetFrame_COG_to_abs(), getKnuckleMass(), inertiaKnuckle);

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
}

// -----------------------------------------------------------------------------
// Get the wheel track using the spindle local position.
// -----------------------------------------------------------------------------
double ChToeBarLeafspringAxle::GetTrack() {
    return 2 * getLocation(SPINDLE).y();
}

// -----------------------------------------------------------------------------
// Return current suspension forces
// -----------------------------------------------------------------------------
std::vector<ChSuspension::ForceTSDA> ChToeBarLeafspringAxle::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces(2);

    forces[0] = ChSuspension::ForceTSDA("Spring", m_spring[side]->GetForce(), m_spring[side]->GetLength(),
                                        m_spring[side]->GetVelocity());
    forces[1] = ChSuspension::ForceTSDA("Shock", m_shock[side]->GetForce(), m_shock[side]->GetLength(),
                                        m_shock[side]->GetVelocity());

    return forces;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarLeafspringAxle::LogHardpointLocations(const ChVector<>& ref, bool inches) {
    double unit = inches ? 1 / 0.0254 : 1.0;

    for (int i = 0; i < NUM_POINTS; i++) {
        ChVector<> pos = ref + unit * getLocation(static_cast<PointId>(i));

        GetLog() << "   " << m_pointNames[i].c_str() << "  " << pos.x() << "  " << pos.y() << "  " << pos.z() << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarLeafspringAxle::LogConstraintViolations(VehicleSide side) {
    // TODO: Update this to reflect new suspension joints
    // Revolute joints
    {
        ChVectorDynamic<> C = m_revoluteKingpin[side]->GetConstraintViolation();
        GetLog() << "Kingpin revolute      ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "  ";
        GetLog() << "  " << C(4) << "\n";
    }

    {
        ChVectorDynamic<> C = m_sphericalTierod->GetConstraintViolation();
        GetLog() << "Tierod spherical          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }
    {
        ChVectorDynamic<> C = m_sphericalDraglink->GetConstraintViolation();
        GetLog() << "Draglink spherical          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "\n";
    }

    {
        ChVectorDynamic<> C = m_universalTierod->GetConstraintViolation();
        GetLog() << "Tierod universal          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }
    {
        ChVectorDynamic<> C = m_universalDraglink->GetConstraintViolation();
        GetLog() << "Draglink universal          ";
        GetLog() << "  " << C(0) << "  ";
        GetLog() << "  " << C(1) << "  ";
        GetLog() << "  " << C(2) << "  ";
        GetLog() << "  " << C(3) << "\n";
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarLeafspringAxle::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    AddVisualizationLink(m_axleTube, m_axleOuterL, m_axleOuterR, getAxleTubeRadius(), ChColor(0.7f, 0.7f, 0.7f));

    AddVisualizationLink(m_tierod, m_tierodOuterL, m_tierodOuterR, getTierodRadius(), ChColor(0.7f, 0.7f, 0.7f));

    if (m_left_knuckle_steers) {
        AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_C], m_pointsL[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    } else {
        AddVisualizationLink(m_draglink, m_pointsL[DRAGLINK_C], m_pointsR[KNUCKLE_DRL], getDraglinkRadius(),
                             ChColor(0.7f, 0.7f, 0.7f));
    }

    AddVisualizationKnuckle(m_knuckle[LEFT], m_pointsL[KNUCKLE_U], m_pointsL[KNUCKLE_L], m_pointsL[TIEROD_K],
                            getKnuckleRadius());
    AddVisualizationKnuckle(m_knuckle[RIGHT], m_pointsR[KNUCKLE_U], m_pointsR[KNUCKLE_L], m_pointsR[TIEROD_K],
                            getKnuckleRadius());

    // Add visualization for the springs and shocks
    m_spring[LEFT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_spring[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.06, 150, 15));
    m_shock[LEFT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
    m_shock[RIGHT]->AddVisualShape(chrono_types::make_shared<ChSegmentShape>());
}

void ChToeBarLeafspringAxle::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_axleTube);
    ChPart::RemoveVisualizationAssets(m_tierod);
    ChPart::RemoveVisualizationAssets(m_draglink);

    ChPart::RemoveVisualizationAssets(m_knuckle[LEFT]);
    ChPart::RemoveVisualizationAssets(m_knuckle[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_spring[LEFT]);
    ChPart::RemoveVisualizationAssets(m_spring[RIGHT]);

    ChPart::RemoveVisualizationAssets(m_shock[LEFT]);
    ChPart::RemoveVisualizationAssets(m_shock[RIGHT]);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarLeafspringAxle::AddVisualizationLink(std::shared_ptr<ChBody> body,
                                                  const ChVector<> pt_1,
                                                  const ChVector<> pt_2,
                                                  double radius,
                                                  const ChColor& color) {
    // Express hardpoint locations in body frame.
    ChVector<> p_1 = body->TransformPointParentToLocal(pt_1);
    ChVector<> p_2 = body->TransformPointParentToLocal(pt_2);

    ChVehicleGeometry::AddVisualizationCylinder(body, p_1, p_2, radius);
}

void ChToeBarLeafspringAxle::AddVisualizationKnuckle(std::shared_ptr<ChBody> knuckle,
                                                     const ChVector<> pt_U,
                                                     const ChVector<> pt_L,
                                                     const ChVector<> pt_T,
                                                     double radius) {
    static const double threshold2 = 1e-6;

    // Express hardpoint locations in body frame.
    ChVector<> p_U = knuckle->TransformPointParentToLocal(pt_U);
    ChVector<> p_L = knuckle->TransformPointParentToLocal(pt_L);
    ChVector<> p_T = knuckle->TransformPointParentToLocal(pt_T);

    if (p_L.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(knuckle, p_L, VNULL, radius);
    }

    if (p_U.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(knuckle, p_U, VNULL, radius);
    }

    if (p_T.Length2() > threshold2) {
        ChVehicleGeometry::AddVisualizationCylinder(knuckle, p_T, VNULL, radius);
    }
}
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void ChToeBarLeafspringAxle::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    bodies.push_back(m_tierod);
    bodies.push_back(m_draglink);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalTierod);
    joints.push_back(m_sphericalDraglink);
    joints.push_back(m_universalDraglink);
    joints.push_back(m_universalTierod);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    ExportJointList(jsonDocument, joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    ExportLinSpringList(jsonDocument, springs);
}

void ChToeBarLeafspringAxle::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    bodies.push_back(m_axleTube);
    bodies.push_back(m_tierod);
    bodies.push_back(m_draglink);
    bodies.push_back(m_knuckle[0]);
    bodies.push_back(m_knuckle[1]);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    joints.push_back(m_sphericalTierod);
    joints.push_back(m_sphericalDraglink);
    joints.push_back(m_universalDraglink);
    joints.push_back(m_universalTierod);
    joints.push_back(m_revoluteKingpin[0]);
    joints.push_back(m_revoluteKingpin[1]);
    database.WriteJoints(joints);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    springs.push_back(m_spring[0]);
    springs.push_back(m_spring[1]);
    springs.push_back(m_shock[0]);
    springs.push_back(m_shock[1]);
    database.WriteLinSprings(springs);
}

}  // end namespace vehicle
}  // end namespace chrono
