// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
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
// Base class for a generic wheeled vehicle suspension. Except for the spindle
// bodies and the associated revolute joints, the topology of such a suspension
// is completely arbitrary and left to derived classes.
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

#include "chrono_vehicle/wheeled_vehicle/suspension/ChGenericWheeledSuspension.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChGenericWheeledSuspension::ChGenericWheeledSuspension(const std::string& name) : ChSuspension(name) {}

ChGenericWheeledSuspension::~ChGenericWheeledSuspension() {
    if (!m_initialized)
        return;

    auto sys = m_spindle[0]->GetSystem();
    if (!sys)
        return;

    for (auto& item : m_bodies)
        sys->Remove(item.second.body);
    for (auto& item : m_joints)
        ChChassis::RemoveJoint(item.second.joint);
    for (auto& item : m_dists)
        sys->Remove(item.second.dist);
    for (auto& item : m_tsdas)
        sys->Remove(item.second.tsda);
    for (auto& item : m_rsdas)
        sys->Remove(item.second.rsda);

    m_bodies.clear();
    m_joints.clear();
    m_dists.clear();
    m_tsdas.clear();
    m_rsdas.clear();
}

// -----------------------------------------------------------------------------

utils::ChBodyGeometry TransformVehicleGeometry(const utils::ChBodyGeometry& geom, int side) {
    static const ChColor colorL(0.38f, 0.74f, 0.36f);
    static const ChColor colorR(0.74f, 0.38f, 0.36f);

    utils::ChBodyGeometry g = geom;
    if (side == VehicleSide::LEFT) {
        g.color_boxes = colorL;
        g.color_spheres = colorL;
        g.color_cylinders = colorL;
    } else {
        g.color_boxes = colorR;
        g.color_spheres = colorR;
        g.color_cylinders = colorR;

        for (auto& s : g.vis_spheres) {
            s.pos.y() *= -1;
        }
        for (auto& c : g.vis_cylinders) {
            c.pos.y() *= -1;
            auto rot = c.rot;
            ChVector3d u = rot.GetAxisX();
            ChVector3d w = rot.GetAxisZ();
            u.y() *= -1;
            w.y() *= -1;
            ChVector3d v = Vcross(w, u);
            ChMatrix33<> R(u, v, w);
            c.rot = R.GetQuaternion();
        }
        for (auto& b : g.vis_boxes) {
            b.pos.y() *= -1;
            auto rot = b.rot;
            ChVector3d u = rot.GetAxisX();
            ChVector3d w = rot.GetAxisZ();
            u.y() *= -1;
            w.y() *= -1;
            ChVector3d v = Vcross(w, u);
            ChMatrix33<> R(u, v, w);
            b.rot = R.GetQuaternion();
        }
    }

    return g;
}

utils::ChTSDAGeometry TransformTSDAGeometry(const utils::ChTSDAGeometry& geom, int side) {
    static const ChColor colorL(0.3f, 0.74f, 0.20f);
    static const ChColor colorR(0.74f, 0.3f, 0.20f);

    utils::ChTSDAGeometry g = geom;
    if (side == VehicleSide::LEFT) {
        g.color = colorL;
    } else {
        g.color = colorR;
    }

    return g;
}

void ChGenericWheeledSuspension::DefineBody(const std::string& name,
                                            bool mirrored,
                                            const ChVector3d& pos,
                                            const ChQuaternion<>& rot,
                                            double mass,
                                            const ChVector3d& inertia_moments,
                                            const ChVector3d& inertia_products,
                                            std::shared_ptr<utils::ChBodyGeometry> geometry) {
    Body b;
    b.body = nullptr;
    b.pos = pos;
    b.rot = rot.GetNormalized();
    b.mass = mass;
    b.inertia_moments = inertia_moments;
    b.inertia_products = inertia_products;

    if (!mirrored) {
        if (geometry)
            b.geometry = *geometry;
        m_bodies.insert({{name, -1}, b});
    } else {
        if (geometry)
            b.geometry = TransformVehicleGeometry(*geometry, 0);
        m_bodies.insert({{name, 0}, b});
        if (geometry)
            b.geometry = TransformVehicleGeometry(*geometry, 1);
        m_bodies.insert({{name, 1}, b});
    }
}

void ChGenericWheeledSuspension::DefineJoint(const std::string& name,
                                             bool mirrored,
                                             ChVehicleJoint::Type type,
                                             BodyIdentifier body1,
                                             BodyIdentifier body2,
                                             const ChVector3d& pos,
                                             const ChQuaternion<>& rot,
                                             std::shared_ptr<ChVehicleBushingData> bdata) {
    Joint j;
    j.joint = nullptr;
    j.type = type;
    j.body1 = body1;
    j.body2 = body2;
    j.pos = pos;
    j.rot = rot.GetNormalized();
    j.bdata = bdata;

    if (!mirrored) {
        m_joints.insert({{name, -1}, j});
    } else {
        m_joints.insert({{name, 0}, j});
        m_joints.insert({{name, 1}, j});
    }
}

void ChGenericWheeledSuspension::DefineDistanceConstraint(const std::string& name,
                                                          bool mirrored,
                                                          BodyIdentifier body1,
                                                          BodyIdentifier body2,
                                                          const ChVector3d& point1,
                                                          const ChVector3d& point2) {
    DistanceConstraint d;
    d.dist = nullptr;
    d.body1 = body1;
    d.body2 = body2;
    d.point1 = point1;
    d.point2 = point2;

    if (!mirrored) {
        m_dists.insert({{name, -1}, d});
    } else {
        m_dists.insert({{name, 0}, d});
        m_dists.insert({{name, 1}, d});
    }
}

void ChGenericWheeledSuspension::DefineTSDA(const std::string& name,
                                            bool mirrored,
                                            BodyIdentifier body1,
                                            BodyIdentifier body2,
                                            const ChVector3d& point1,
                                            const ChVector3d& point2,
                                            double rest_length,
                                            std::shared_ptr<ChLinkTSDA::ForceFunctor> force,
                                            std::shared_ptr<utils::ChTSDAGeometry> geometry) {
    TSDA t;
    t.tsda = nullptr;
    t.body1 = body1;
    t.body2 = body2;
    t.point1 = point1;
    t.point2 = point2;
    t.rest_length = rest_length;
    t.force = force;

    if (!mirrored) {
        if (geometry)
            t.geometry = *geometry;
        m_tsdas.insert({{name, -1}, t});
    } else {
        if (geometry)
            t.geometry = TransformTSDAGeometry(*geometry, 0);
        m_tsdas.insert({{name, 0}, t});
        if (geometry)
            t.geometry = TransformTSDAGeometry(*geometry, 1);
        m_tsdas.insert({{name, 1}, t});
    }
}

void ChGenericWheeledSuspension::DefineRSDA(const std::string& name,
                                            bool mirrored,
                                            BodyIdentifier body1,
                                            BodyIdentifier body2,
                                            const ChVector3d& pos,
                                            const ChVector3d& axis,
                                            double rest_angle,
                                            std::shared_ptr<ChLinkRSDA::TorqueFunctor> torque) {
    RSDA r;
    r.rsda = nullptr;
    r.body1 = body1;
    r.body2 = body2;
    r.pos = pos;
    r.axis = axis.GetNormalized();
    r.rest_angle = rest_angle;
    r.torque = torque;

    if (!mirrored) {
        m_rsdas.insert({{name, -1}, r});
    } else {
        m_rsdas.insert({{name, 0}, r});
        r.axis *= ChVector3d(-1, 0, -1);
        m_rsdas.insert({{name, 1}, r});
    }
}

// -----------------------------------------------------------------------------

std::string ChGenericWheeledSuspension::Name(const PartKey& id) const {
    if (id.side == -1)
        return m_name + "_" + id.name;

    return m_name + "_" + id.name + (id.side == VehicleSide::LEFT ? "_L" : "_R");
}

ChVector3d ChGenericWheeledSuspension::TransformPosition(const ChVector3d& pos_loc, int side) const {
    ChVector3d pos = pos_loc;
    if (side == VehicleSide::RIGHT)
        pos.y() = -pos.y();
    return m_X_SA.TransformPointLocalToParent(pos);
}

ChVector3d ChGenericWheeledSuspension::TransformDirection(const ChVector3d& dir_loc, int side) const {
    ChVector3d dir = dir_loc;
    if (side == VehicleSide::RIGHT)
        dir.y() = -dir.y();
    return m_X_SA.TransformDirectionLocalToParent(dir);
}

ChQuaternion<> ChGenericWheeledSuspension::TransformRotation(const ChQuaternion<>& rot_local, int side) const {
    ChQuaternion<> rot = rot_local.GetNormalized();
    if (side == VehicleSide::RIGHT) {
        ChVector3d u = rot.GetAxisX();
        ChVector3d w = rot.GetAxisZ();
        u.y() *= -1;
        w.y() *= -1;
        ChVector3d v = Vcross(w, u);
        ChMatrix33<> R(u, v, w);
        rot = R.GetQuaternion();
    }
    return m_X_SA.GetRot() * rot;
}

std::shared_ptr<ChBody> ChGenericWheeledSuspension::FindBody(BodyIdentifier body, int side) const {
    // When looking for a body, there are a couple of scenarios to consider:
    // 1) The body identifier only holds a name and no side (-1) which means we will look for a body
    //    with this name as either:
    //    a) an unmirrored body (side is -1) or
    //    b) as one on the specified side of the suspension (LEFT, RIGHT).
    // 2) The body identifier holds a name and a side (LEFT, RIGHT) which means we will look for a
    //    body with that name on that side, regardless of what side of the suspension (if any) we
    //    are on.
    if (body.side == -1) {
        auto b1 = m_bodies.find({body.name, -1});
        if (b1 != m_bodies.end()) {
            return b1->second.body;
        }
        auto b2 = m_bodies.find({body.name, side});
        assert(b2 != m_bodies.end());
        return b2->second.body;
    } else {
        auto b1 = m_bodies.find({body.name, body.side});
        assert(b1 != m_bodies.end());
        return b1->second.body;
    }
}

void ChGenericWheeledSuspension::Initialize(std::shared_ptr<ChChassis> chassis,
                                            std::shared_ptr<ChSubchassis> subchassis,
                                            std::shared_ptr<ChSteering> steering,
                                            const ChVector3d& location,
                                            double left_ang_vel,
                                            double right_ang_vel) {
    ChSuspension::Initialize(chassis, subchassis, steering, location, left_ang_vel, right_ang_vel);

    m_parent = chassis;
    m_rel_loc = location;

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrameRefToAbs().GetRot();

    // Express the suspension reference frame in the absolute coordinate system.
    m_X_SA.SetPos(location);
    m_X_SA.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    // Initialize all bodies in the suspension subsystem
    for (auto& item : m_bodies) {
        ChVector3d pos = TransformPosition(item.second.pos, item.first.side);
        ChQuaternion<> rot = TransformRotation(item.second.rot, item.first.side);
        item.second.body = chrono_types::make_shared<ChBody>();
        item.second.body->SetMass(item.second.mass);
        item.second.body->SetInertiaXX(item.second.inertia_moments);
        item.second.body->SetInertiaXY(item.second.inertia_products);
        item.second.body->SetPos(pos);
        item.second.body->SetRot(rot);
        item.second.body->SetName(Name(item.first));
        chassis->GetSystem()->AddBody(item.second.body);
    }

    // Create and initialize joints in the suspension subsystem
    for (auto& item : m_joints) {
        // Extract the two bodies connected by this joint
        std::shared_ptr<ChBody> body1;
        if (item.second.body1.chassis)
            body1 = chassis->GetBody();
        else if (item.second.body1.subchassis && subchassis != nullptr)
            body1 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body1.steering && steering != nullptr)
            body1 = steering->GetSteeringLink();
        else
            body1 = FindBody(item.second.body1, item.first.side);

        std::shared_ptr<ChBody> body2;
        if (item.second.body2.chassis)
            body2 = chassis->GetBody();
        else if (item.second.body2.subchassis && subchassis != nullptr)
            body2 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body2.steering && steering != nullptr)
            body2 = steering->GetSteeringLink();
        else
            body2 = FindBody(item.second.body2, item.first.side);

        // Create joint
        ChVector3d pos = TransformPosition(item.second.pos, item.first.side);
        ChQuaternion<> rot = TransformRotation(item.second.rot, item.first.side);
        item.second.joint = chrono_types::make_shared<ChVehicleJoint>(item.second.type,     //
                                                                      Name(item.first),     //
                                                                      body1,                //
                                                                      body2,                //
                                                                      ChFrame<>(pos, rot),  //
                                                                      item.second.bdata);
        chassis->AddJoint(item.second.joint);
    }

    // Create and initialize distance constraints in the suspension subsystem
    for (auto& item : m_dists) {
        // Extract the two bodies connected by this constraint
        std::shared_ptr<ChBody> body1;
        if (item.second.body1.chassis)
            body1 = chassis->GetBody();
        else if (item.second.body1.subchassis && subchassis != nullptr)
            body1 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body1.steering && steering != nullptr)
            body1 = steering->GetSteeringLink();
        else
            body1 = FindBody(item.second.body1, item.first.side);

        std::shared_ptr<ChBody> body2;
        if (item.second.body2.chassis)
            body2 = chassis->GetBody();
        else if (item.second.body2.subchassis && subchassis != nullptr)
            body2 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body2.steering && steering != nullptr)
            body2 = steering->GetSteeringLink();
        else
            body2 = FindBody(item.second.body2, item.first.side);

        // Create distance constraint
        ChVector3d point1 = TransformPosition(item.second.point1, item.first.side);
        ChVector3d point2 = TransformPosition(item.second.point2, item.first.side);
        item.second.dist = chrono_types::make_shared<ChLinkDistance>();
        item.second.dist->SetName(Name(item.first));
        item.second.dist->Initialize(body1, body2, false, point1, point2);
        chassis->GetSystem()->AddLink(item.second.dist);
    }

    // Create and initialize the spindle bodies, the spindle revolute joints, and the suspension axles
    for (int side = 0; side < 2; side++) {
        double ang_vel = (side == LEFT) ? left_ang_vel : right_ang_vel;
        double sign = (side == LEFT) ? -1 : +1;

        auto spindlePos = TransformPosition(getSpindlePos(), side);
        auto spindleRot = chassisRot * QuatFromAngleZ(sign * getToeAngle()) * QuatFromAngleX(sign * getCamberAngle());

        // Spindle body
        m_spindle[side] = chrono_types::make_shared<ChBody>();
        m_spindle[side]->SetName(Name({"spindle", side}));
        m_spindle[side]->SetPos(spindlePos);
        m_spindle[side]->SetRot(spindleRot);
        m_spindle[side]->SetAngVelLocal(ChVector3d(0, ang_vel, 0));
        m_spindle[side]->SetMass(getSpindleMass());
        m_spindle[side]->SetInertiaXX(getSpindleInertia());
        chassis->GetSystem()->AddBody(m_spindle[side]);

        // Spindle revolute joint
        auto abody_id = getSpindleAttachmentBody();
        std::shared_ptr<ChBody> abody;
        if (abody_id.chassis)
            abody = chassis->GetBody();
        else
            abody = FindBody(abody_id, side);

        m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
        m_revolute[side]->SetName(Name({"spindle_rev", side}));
        m_revolute[side]->Initialize(m_spindle[side], abody,
                                     ChFrame<>(spindlePos, spindleRot * QuatFromAngleX(CH_PI_2)));
        chassis->GetSystem()->AddLink(m_revolute[side]);

        // Axle shaft
        m_axle[side] = chrono_types::make_shared<ChShaft>();
        m_axle[side]->SetName(Name({"axle", side}));
        m_axle[side]->SetInertia(getAxleInertia());
        m_axle[side]->SetPosDt(-ang_vel);
        chassis->GetSystem()->AddShaft(m_axle[side]);

        // Axle connection to spindle
        m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftBodyRotation>();
        m_axle_to_spindle[side]->SetName(Name({"axle_to_spindle", side}));
        m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector3d(0, -1, 0));
        chassis->GetSystem()->Add(m_axle_to_spindle[side]);
    }

    // Create and initialize TSDAs in the suspension subsystem
    for (auto& item : m_tsdas) {
        // Extract the two bodies connected by this element
        std::shared_ptr<ChBody> body1;
        if (item.second.body1.chassis)
            body1 = chassis->GetBody();
        else if (item.second.body1.subchassis && subchassis != nullptr)
            body1 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body1.steering && steering != nullptr)
            body1 = steering->GetSteeringLink();
        else
            body1 = FindBody(item.second.body1, item.first.side);

        std::shared_ptr<ChBody> body2;
        if (item.second.body2.chassis)
            body2 = chassis->GetBody();
        else if (item.second.body2.subchassis && subchassis != nullptr)
            body2 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body2.steering && steering != nullptr)
            body2 = steering->GetSteeringLink();
        else
            body2 = FindBody(item.second.body2, item.first.side);

        // Create TSDA
        ChVector3d point1 = TransformPosition(item.second.point1, item.first.side);
        ChVector3d point2 = TransformPosition(item.second.point2, item.first.side);
        item.second.tsda = chrono_types::make_shared<ChLinkTSDA>();
        item.second.tsda->SetName(Name(item.first));
        item.second.tsda->Initialize(body1, body2, false, point1, point2);
        item.second.tsda->SetRestLength(item.second.rest_length);
        item.second.tsda->RegisterForceFunctor(item.second.force);
        chassis->GetSystem()->AddLink(item.second.tsda);
    }

    // Create and initialize RSDAs in the suspension subsystem
    for (auto& item : m_rsdas) {
        // Extract the two bodies connected by this element
        std::shared_ptr<ChBody> body1;
        if (item.second.body1.chassis)
            body1 = chassis->GetBody();
        else if (item.second.body1.subchassis && subchassis != nullptr)
            body1 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body1.steering && steering != nullptr)
            body1 = steering->GetSteeringLink();
        else
            body1 = FindBody(item.second.body1, item.first.side);

        std::shared_ptr<ChBody> body2;
        if (item.second.body2.chassis)
            body2 = chassis->GetBody();
        else if (item.second.body2.subchassis && subchassis != nullptr)
            body2 = subchassis->GetBeam(item.first.side == 0 ? LEFT : RIGHT);
        else if (item.second.body2.steering && steering != nullptr)
            body2 = steering->GetSteeringLink();
        else
            body2 = FindBody(item.second.body2, item.first.side);

        // Create RSDA
        ChVector3d pos = TransformPosition(item.second.pos, item.first.side);
        ChVector3d axis = TransformDirection(item.second.axis, item.first.side);
        ChMatrix33<> rot;
        rot.SetFromAxisX(axis);
        ChQuaternion<> quat = rot.GetQuaternion() * QuatFromAngleY(CH_PI_2);
        item.second.rsda = chrono_types::make_shared<ChLinkRSDA>();
        item.second.rsda->SetName(Name(item.first));
        item.second.rsda->Initialize(body1, body2, ChFrame<>(pos, quat));
        item.second.rsda->SetRestAngle(item.second.rest_angle);
        item.second.rsda->RegisterTorqueFunctor(item.second.torque);
        chassis->GetSystem()->AddLink(item.second.rsda);
    }
}

void ChGenericWheeledSuspension::InitializeInertiaProperties() {
    m_mass = 2 * getSpindleMass();
    for (const auto& item : m_bodies)
        m_mass += item.second.body->GetMass();
}

void ChGenericWheeledSuspension::UpdateInertiaProperties() {
    m_xform = m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT));

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;

    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    composite.AddComponent(m_spindle[LEFT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrameCOMToAbs(), getSpindleMass(), inertiaSpindle);
    for (const auto& item : m_bodies)
        composite.AddComponent(item.second.body->GetFrameCOMToAbs(), item.second.body->GetMass(),
                               item.second.body->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.SetPos(m_xform.TransformPointParentToLocal(composite.GetCOM()));
    m_com.SetRot(QUNIT);

    m_inertia = m_xform.GetRotMat().transpose() * composite.GetInertia() * m_xform.GetRotMat();
}

// -----------------------------------------------------------------------------

double ChGenericWheeledSuspension::GetTrack() {
    return 2 * getSpindlePos().y();
}

std::shared_ptr<ChBody> ChGenericWheeledSuspension::GetAntirollBody(VehicleSide side) const {
    auto abody_id = getAntirollBody();
    return FindBody(abody_id, side);
}

// -----------------------------------------------------------------------------

std::vector<ChSuspension::ForceTSDA> ChGenericWheeledSuspension::ReportSuspensionForce(VehicleSide side) const {
    std::vector<ChSuspension::ForceTSDA> forces;

    for (const auto& item : m_tsdas) {
        if (item.first.side != side)
            continue;
        auto tsda = item.second.tsda;
        ////std::cout << "TSDA " << item.first.name << "\n";
        ////std::cout << "  force:    " << tsda->GetForce() << "\n";
        ////std::cout << "  length:   " << tsda->GetLength() << "\n";
        ////std::cout << "  velocity: " << tsda->GetVelocity() << "\n";
        forces.push_back(
            ChSuspension::ForceTSDA(item.first.name, tsda->GetForce(), tsda->GetLength(), tsda->GetVelocity()));
    }

    return forces;
}

std::vector<ChSuspension::ForceRSDA> ChGenericWheeledSuspension::ReportSuspensionTorque(VehicleSide side) const {
    std::vector<ChSuspension::ForceRSDA> torques;

    for (const auto& item : m_rsdas) {
        if (item.first.side != side)
            continue;
        auto rsda = item.second.rsda;
        torques.push_back(
            ChSuspension::ForceRSDA(item.first.name, rsda->GetTorque(), rsda->GetAngle(), rsda->GetVelocity()));
    }

    return torques;
}

void ChGenericWheeledSuspension::LogConstraintViolations(VehicleSide side) {
    // Spindle revolute joint
    {
        const auto& C = m_revolute[side]->GetConstraintViolation();
        std::cout << "Spindle revolute "
                  << "\n";
        std::cout << "   " << C.transpose() << "\n";
    }

    // Suspension joints
    for (const auto& item : m_joints) {
        if (item.first.side != side)
            continue;
        const auto& joint = item.second.joint;
        if (!joint->IsKinematic())
            continue;
        auto link = joint->GetAsLink();
        const auto& C = link->GetConstraintViolation();
        assert(C.size() == link->GetNumConstraintsBilateral());
        std::cout << "Joint " << item.first.name << " type: " << ChVehicleJoint::GetTypeString(item.second.type)
                  << "\n";
        std::cout << "   " << C.transpose() << "\n";
    }

    // Distance constraints
    for (const auto& item : m_dists) {
        if (item.first.side != side)
            continue;
        const auto& dist = item.second.dist;
        const auto& C = dist->GetConstraintViolation();
        std::cout << "Distance constraint " << item.first.name << "\n";
        std::cout << "   " << C.transpose() << "\n";
    }
}

// -----------------------------------------------------------------------------

void ChGenericWheeledSuspension::AddVisualizationAssets(VisualizationType vis) {
    ChSuspension::AddVisualizationAssets(vis);

    if (vis == VisualizationType::NONE)
        return;

    for (auto& item : m_bodies)
        item.second.geometry.CreateVisualizationAssets(item.second.body, vis);
    for (auto& item : m_tsdas)
        item.second.geometry.CreateVisualizationAssets(item.second.tsda);
    for (auto& item : m_dists)
        item.second.dist->AddVisualShape(chrono_types::make_shared<ChVisualShapeSegment>());
}

void ChGenericWheeledSuspension::RemoveVisualizationAssets() {
    for (const auto& item : m_bodies)
        ChPart::RemoveVisualizationAssets(item.second.body);
    for (const auto& item : m_tsdas)
        ChPart::RemoveVisualizationAssets(item.second.tsda);
    for (const auto& item : m_dists)
        ChPart::RemoveVisualizationAssets(item.second.dist);

    ChSuspension::RemoveVisualizationAssets();
}

// -----------------------------------------------------------------------------

void ChGenericWheeledSuspension::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    for (const auto& item : m_bodies)
        bodies.push_back(item.second.body);
    ExportBodyList(jsonDocument, bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    ExportShaftList(jsonDocument, shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    for (const auto& item : m_joints)
        item.second.joint->IsKinematic() ? joints.push_back(item.second.joint->GetAsLink())
                                         : bushings.push_back(item.second.joint->GetAsBushing());
    for (const auto& item : m_dists)
        joints.push_back(item.second.dist);
    ExportJointList(jsonDocument, joints);
    ExportBodyLoadList(jsonDocument, bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    for (const auto& item : m_tsdas)
        springs.push_back(item.second.tsda);
    ExportLinSpringList(jsonDocument, springs);

    std::vector<std::shared_ptr<ChLinkRSDA>> rot_springs;
    for (const auto& item : m_rsdas)
        rot_springs.push_back(item.second.rsda);
    ExportRotSpringList(jsonDocument, rot_springs);
}

void ChGenericWheeledSuspension::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_spindle[0]);
    bodies.push_back(m_spindle[1]);
    for (const auto& item : m_bodies)
        bodies.push_back(item.second.body);
    database.WriteBodies(bodies);

    std::vector<std::shared_ptr<ChShaft>> shafts;
    shafts.push_back(m_axle[0]);
    shafts.push_back(m_axle[1]);
    database.WriteShafts(shafts);

    std::vector<std::shared_ptr<ChLink>> joints;
    std::vector<std::shared_ptr<ChLoadBodyBody>> bushings;
    joints.push_back(m_revolute[0]);
    joints.push_back(m_revolute[1]);
    for (const auto& item : m_joints)
        item.second.joint->IsKinematic() ? joints.push_back(item.second.joint->GetAsLink())
                                         : bushings.push_back(item.second.joint->GetAsBushing());
    for (const auto& item : m_dists)
        joints.push_back(item.second.dist);
    database.WriteJoints(joints);
    database.WriteBodyLoads(bushings);

    std::vector<std::shared_ptr<ChLinkTSDA>> springs;
    for (const auto& item : m_tsdas)
        springs.push_back(item.second.tsda);
    database.WriteLinSprings(springs);

    std::vector<std::shared_ptr<ChLinkRSDA>> rot_springs;
    for (const auto& item : m_rsdas)
        rot_springs.push_back(item.second.rsda);
    database.WriteRotSprings(rot_springs);
}

// -----------------------------------------------------------------------------

ChGenericWheeledSuspension::BodyIdentifier::BodyIdentifier(const std::string& part_name,
                                                           int vehicle_side,
                                                           bool is_chassis,
                                                           bool is_subchassis,
                                                           bool is_steering)
    : name(part_name), side(vehicle_side), chassis(is_chassis), subchassis(is_subchassis), steering(is_steering) {}

ChGenericWheeledSuspension::BodyIdentifier::BodyIdentifier()
    : name(""), side(-1), chassis(false), subchassis(false), steering(false) {}

ChGenericWheeledSuspension::ChassisIdentifier::ChassisIdentifier() : BodyIdentifier("", -1, true, false, false) {}
ChGenericWheeledSuspension::SubchassisIdentifier::SubchassisIdentifier() : BodyIdentifier("", -1, false, true, false) {}
ChGenericWheeledSuspension::SteeringIdentifier::SteeringIdentifier() : BodyIdentifier("", -1, false, false, true) {}

bool ChGenericWheeledSuspension::PartKey::operator==(const PartKey& rhs) const {
    return name == rhs.name && side == rhs.side;
}

std::size_t ChGenericWheeledSuspension::PartKeyHash::operator()(const PartKey& id) const {
    std::size_t h1 = std::hash<std::string>{}(id.name);
    if (id.side == -1)
        return h1;
    std::string ext = (id.side == VehicleSide::LEFT ? "_L" : "_R");
    std::size_t h2 = std::hash<std::string>{}(ext);
    return h1 ^ (h2 << 1);
}

}  // end namespace vehicle
}  // end namespace chrono
