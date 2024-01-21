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
    auto sys = m_spindle[0]->GetSystem();
    if (sys) {
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
}

// -----------------------------------------------------------------------------

ChVehicleGeometry TransformVehicleGeometry(const ChVehicleGeometry& geom, int side) {
    static const ChColor colorL(0.38f, 0.74f, 0.36f);
    static const ChColor colorR(0.74f, 0.38f, 0.36f);

    ChVehicleGeometry g = geom;
    g.m_has_colors = true;
    if (side == VehicleSide::LEFT) {
        g.m_color_boxes = colorL;
        g.m_color_spheres = colorL;
        g.m_color_cylinders = colorL;
    } else {
        g.m_color_boxes = colorR;
        g.m_color_spheres = colorR;
        g.m_color_cylinders = colorR;

        for (auto& s : g.m_vis_spheres) {
            s.m_pos.y() *= -1;
        }
        for (auto& c : g.m_vis_cylinders) {
            c.m_pos.y() *= -1;
            auto rot = c.m_rot;
            ChVector<> u = rot.GetXaxis();
            ChVector<> w = rot.GetZaxis();
            u.y() *= -1;
            w.y() *= -1;
            ChVector<> v = Vcross(w, u);
            ChMatrix33<> R(u, v, w);
            c.m_rot = R.Get_A_quaternion();
        }
        for (auto& b : g.m_vis_boxes) {
            b.m_pos.y() *= -1;
            auto rot = b.m_rot;
            ChVector<> u = rot.GetXaxis();
            ChVector<> w = rot.GetZaxis();
            u.y() *= -1;
            w.y() *= -1;
            ChVector<> v = Vcross(w, u);
            ChMatrix33<> R(u, v, w);
            b.m_rot = R.Get_A_quaternion();
        }
    }

    return g;
}

ChTSDAGeometry TransformTSDAGeometry(const ChTSDAGeometry& geom, int side) {
    static const ChColor colorL(0.3f, 0.74f, 0.20f);
    static const ChColor colorR(0.74f, 0.3f, 0.20f);

    ChTSDAGeometry g = geom;
    g.m_has_color = true;
    if (side == VehicleSide::LEFT) {
        g.m_color = colorL;
    } else {
        g.m_color = colorR;
    }

    return g;
}

void ChGenericWheeledSuspension::DefineBody(const std::string& name,
                                            bool mirrored,
                                            const ChVector<>& pos,
                                            const ChQuaternion<>& rot,
                                            double mass,
                                            const ChVector<>& inertia_moments,
                                            const ChVector<>& inertia_products,
                                            std::shared_ptr<ChVehicleGeometry> geometry) {
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
                                             const ChVector<>& pos,
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
                                                          const ChVector<>& point1,
                                                          const ChVector<>& point2) {
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
                                            const ChVector<>& point1,
                                            const ChVector<>& point2,
                                            double rest_length,
                                            std::shared_ptr<ChLinkTSDA::ForceFunctor> force,
                                            std::shared_ptr<ChTSDAGeometry> geometry) {
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
                                            const ChVector<>& pos,
                                            const ChVector<>& axis,
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
        r.axis *= ChVector<>(-1, 0, -1);
        m_rsdas.insert({{name, 1}, r});
    }
}

// -----------------------------------------------------------------------------

std::string ChGenericWheeledSuspension::Name(const PartKey& id) const {
    if (id.side == -1)
        return m_name + "_" + id.name;

    return m_name + "_" + id.name + (id.side == VehicleSide::LEFT ? "_L" : "_R");
}

ChVector<> ChGenericWheeledSuspension::TransformPosition(const ChVector<>& pos_loc, int side) const {
    ChVector<> pos = pos_loc;
    if (side == VehicleSide::RIGHT)
        pos.y() = -pos.y();
    return m_X_SA.TransformPointLocalToParent(pos);
}

ChVector<> ChGenericWheeledSuspension::TransformDirection(const ChVector<>& dir_loc, int side) const {
    ChVector<> dir = dir_loc;
    if (side == VehicleSide::RIGHT)
        dir.y() = -dir.y();
    return m_X_SA.TransformDirectionLocalToParent(dir);
}

ChQuaternion<> ChGenericWheeledSuspension::TransformRotation(const ChQuaternion<>& rot_local, int side) const {
    ChQuaternion<> rot = rot_local.GetNormalized();
    if (side == VehicleSide::RIGHT) {
        ChVector<> u = rot.GetXaxis();
        ChVector<> w = rot.GetZaxis();
        u.y() *= -1;
        w.y() *= -1;
        ChVector<> v = Vcross(w, u);
        ChMatrix33<> R(u, v, w);
        rot = R.Get_A_quaternion();
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
                                            const ChVector<>& location,
                                            double left_ang_vel,
                                            double right_ang_vel) {
    ChSuspension::Initialize(chassis, subchassis, steering, location, left_ang_vel, right_ang_vel);

    m_parent = chassis;
    m_rel_loc = location;

    // Chassis orientation (expressed in absolute frame)
    // Recall that the suspension reference frame is aligned with the chassis.
    ChQuaternion<> chassisRot = chassis->GetBody()->GetFrame_REF_to_abs().GetRot();

    // Express the suspension reference frame in the absolute coordinate system.
    m_X_SA.SetPos(location);
    m_X_SA.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    // Initialize all bodies in the suspension subsystem
    for (auto& item : m_bodies) {
        ChVector<> pos = TransformPosition(item.second.pos, item.first.side);
        ChQuaternion<> rot = TransformRotation(item.second.rot, item.first.side);
        item.second.body = chrono_types::make_shared<ChBody>();
        item.second.body->SetMass(item.second.mass);
        item.second.body->SetInertiaXX(item.second.inertia_moments);
        item.second.body->SetInertiaXY(item.second.inertia_products);
        item.second.body->SetPos(pos);
        item.second.body->SetRot(rot);
        item.second.body->SetNameString(Name(item.first));
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
        ChVector<> pos = TransformPosition(item.second.pos, item.first.side);
        ChQuaternion<> rot = TransformRotation(item.second.rot, item.first.side);
        item.second.joint = chrono_types::make_shared<ChVehicleJoint>(item.second.type,        //
                                                                      Name(item.first),        //
                                                                      body1,                   //
                                                                      body2,                   //
                                                                      ChCoordsys<>(pos, rot),  //
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
        ChVector<> point1 = TransformPosition(item.second.point1, item.first.side);
        ChVector<> point2 = TransformPosition(item.second.point2, item.first.side);
        item.second.dist = chrono_types::make_shared<ChLinkDistance>();
        item.second.dist->SetNameString(Name(item.first));
        item.second.dist->Initialize(body1, body2, false, point1, point2);
        chassis->GetSystem()->AddLink(item.second.dist);
    }

    // Create and initialize the spindle bodies, the spindle revolute joints, and the suspension axles
    for (int side = 0; side < 2; side++) {
        double ang_vel = (side == LEFT) ? left_ang_vel : right_ang_vel;
        double sign = (side == LEFT) ? -1 : +1;

        auto spindlePos = TransformPosition(getSpindlePos(), side);
        auto spindleRot = chassisRot * Q_from_AngZ(sign * getToeAngle()) * Q_from_AngX(sign * getCamberAngle());

        // Spindle body
        m_spindle[side] = chrono_types::make_shared<ChBody>();
        m_spindle[side]->SetNameString(Name({"spindle", side}));
        m_spindle[side]->SetPos(spindlePos);
        m_spindle[side]->SetRot(spindleRot);
        m_spindle[side]->SetWvel_loc(ChVector<>(0, ang_vel, 0));
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

        ChCoordsys<> rev_csys(spindlePos, spindleRot * Q_from_AngX(CH_C_PI_2));
        m_revolute[side] = chrono_types::make_shared<ChLinkLockRevolute>();
        m_revolute[side]->SetNameString(Name({"spindle_rev", side}));
        m_revolute[side]->Initialize(m_spindle[side], abody, rev_csys);
        chassis->GetSystem()->AddLink(m_revolute[side]);

        // Axle shaft
        m_axle[side] = chrono_types::make_shared<ChShaft>();
        m_axle[side]->SetNameString(Name({"axle", side}));
        m_axle[side]->SetInertia(getAxleInertia());
        m_axle[side]->SetPos_dt(-ang_vel);
        chassis->GetSystem()->AddShaft(m_axle[side]);

        // Axle connection to spindle
        m_axle_to_spindle[side] = chrono_types::make_shared<ChShaftsBody>();
        m_axle_to_spindle[side]->SetNameString(Name({"axle_to_spindle", side}));
        m_axle_to_spindle[side]->Initialize(m_axle[side], m_spindle[side], ChVector<>(0, -1, 0));
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
        ChVector<> point1 = TransformPosition(item.second.point1, item.first.side);
        ChVector<> point2 = TransformPosition(item.second.point2, item.first.side);
        item.second.tsda = chrono_types::make_shared<ChLinkTSDA>();
        item.second.tsda->SetNameString(Name(item.first));
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
        ChVector<> pos = TransformPosition(item.second.pos, item.first.side);
        ChVector<> axis = TransformDirection(item.second.axis, item.first.side);
        ChMatrix33<> rot;
        rot.Set_A_Xdir(axis);
        ChQuaternion<> quat = rot.Get_A_quaternion() * Q_from_AngY(CH_C_PI_2);
        item.second.rsda = chrono_types::make_shared<ChLinkRSDA>();
        item.second.rsda->SetNameString(Name(item.first));
        item.second.rsda->Initialize(body1, body2, ChCoordsys<>(pos, quat));
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
    m_parent->GetTransform().TransformLocalToParent(ChFrame<>(m_rel_loc, QUNIT), m_xform);

    // Calculate COM and inertia expressed in global frame
    utils::CompositeInertia composite;

    ChMatrix33<> inertiaSpindle(getSpindleInertia());
    composite.AddComponent(m_spindle[LEFT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    composite.AddComponent(m_spindle[RIGHT]->GetFrame_COG_to_abs(), getSpindleMass(), inertiaSpindle);
    for (const auto& item : m_bodies)
        composite.AddComponent(item.second.body->GetFrame_COG_to_abs(), item.second.body->GetMass(),
                               item.second.body->GetInertia());

    // Express COM and inertia in subsystem reference frame
    m_com.coord.pos = m_xform.TransformPointParentToLocal(composite.GetCOM());
    m_com.coord.rot = QUNIT;

    m_inertia = m_xform.GetA().transpose() * composite.GetInertia() * m_xform.GetA();
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
        ////GetLog() << "TSDA " << item.first.name << "\n";
        ////GetLog() << "  force:    " << tsda->GetForce() << "\n";
        ////GetLog() << "  length:   " << tsda->GetLength() << "\n";
        ////GetLog() << "  velocity: " << tsda->GetVelocity() << "\n";
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
        GetLog() << "Spindle revolute " << "\n";
        GetLog() << "   " << C.transpose() << "\n";
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
        assert(C.size() == link->GetDOC_c());
        GetLog() << "Joint " << item.first.name << " type: " << ChVehicleJoint::GetTypeString(item.second.type)
                  << "\n";
        GetLog() << "   " << C.transpose() << "\n";
    }

    // Distance constraints
    for (const auto& item : m_dists) {
        if (item.first.side != side)
            continue;
        const auto& dist = item.second.dist;
        const auto& C = dist->GetConstraintViolation();
        GetLog() << "Distance constraint " << item.first.name << "\n";
        GetLog() << "   " << C.transpose() << "\n";
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
        item.second.geometry.CreateVisualizationAssets(item.second.tsda, vis);
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
