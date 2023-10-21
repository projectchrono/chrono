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
// Base classes for the chassis vehicle subsystems:
//  ChChassis          - base class for a main chassis
//  ChChassisRear      - base class for a rear chassis
//  ChChassisConnector - base class for rear chassis connectors
//
// =============================================================================

#include "chrono/assets/ChSphereShape.h"

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChChassis::ChChassis(const std::string& name, bool fixed) : ChPart(name), m_fixed(fixed) {
    m_bushings = chrono_types::make_shared<ChLoadContainer>();
    m_external = chrono_types::make_shared<ChLoadContainer>();
}

ChChassis::~ChChassis() {
    auto sys = m_body->GetSystem();
    if (sys) {
        sys->Remove(m_body);
        sys->Remove(m_bushings);
        sys->Remove(m_external);
    }
}

// -----------------------------------------------------------------------------
const ChVector<>& ChChassis::GetPos() const {
    return m_body->GetFrame_REF_to_abs().GetPos();
}

ChQuaternion<> ChChassis::GetRot() const {
    return m_body->GetFrame_REF_to_abs().GetRot() * ChWorldFrame::Quaternion();
}

ChVector<> ChChassis::GetPointLocation(const ChVector<>& locpos) const {
    return m_body->GetFrame_REF_to_abs().TransformPointLocalToParent(locpos);
}

ChVector<> ChChassis::GetPointVelocity(const ChVector<>& locpos) const {
    return m_body->GetFrame_REF_to_abs().PointSpeedLocalToParent(locpos);
}

ChVector<> ChChassis::GetPointAcceleration(const ChVector<>& locpos) const {
    ChVector<> acc_abs = m_body->GetFrame_REF_to_abs().PointAccelerationLocalToParent(locpos);
    return m_body->GetFrame_REF_to_abs().TransformDirectionParentToLocal(acc_abs);
}

// Return the global driver position
ChVector<> ChChassis::GetDriverPos() const {
    return m_body->GetFrame_REF_to_abs().TransformPointLocalToParent(GetLocalDriverCoordsys().pos);
}

// Return the speed measured at the origin of the chassis reference frame.
double ChChassis::GetSpeed() const {
    const auto& x_dir = m_body->GetA().Get_A_Xaxis();
    const auto& vel = m_body->GetFrame_REF_to_abs().GetPos_dt();
    return Vdot(vel, x_dir);
}

// Return the speed measured at the chassis center of mass.
double ChChassis::GetCOMSpeed() const {
    const auto& x_dir = m_body->GetA().Get_A_Xaxis();
    const auto& vel = m_body->GetPos_dt();
    return Vdot(vel, x_dir);
}

void ChChassis::Initialize(ChSystem* system,
                           const ChCoordsys<>& chassisPos,
                           double chassisFwdVel,
                           int collision_family) {
    // Initial pose and velocity assumed to be given in current WorldFrame
    ChFrame<> chassis_pos(chassisPos.pos, ChMatrix33<>(chassisPos.rot) * ChWorldFrame::Rotation().transpose());

    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetIdentifier(0);
    m_body->SetNameString(m_name + " body");
    m_body->SetMass(GetBodyMass());
    m_body->SetFrame_COG_to_REF(GetBodyCOMFrame());
    m_body->SetInertia(GetBodyInertia());
    m_body->SetBodyFixed(m_fixed);

    m_body->SetFrame_REF_to_abs(chassis_pos);
    m_body->SetPos_dt(chassisFwdVel * chassis_pos.TransformDirectionLocalToParent(ChVector<>(1, 0, 0)));

    system->Add(m_body);

    // Add containers for bushing elements and external forces.
    system->Add(m_bushings);
    system->Add(m_external);

    // Add pre-defined markers (driver position and COM) on the chassis body.
    AddMarker("driver position", GetLocalDriverCoordsys());
    AddMarker("COM", ChCoordsys<>(GetCOMFrame().GetPos(), GetCOMFrame().GetRot()));

    // Mark as initialized
    m_initialized = true;
}

void ChChassis::AddMarker(const std::string& name, const ChCoordsys<>& pos) {
    // Do nothing if the chassis is not yet initialized
    if (!m_body)
        return;

    // Note: marker local positions are assumed to be relative to the centroidal frame
    //       of the associated body.
    auto pos_com = m_body->GetFrame_REF_to_COG().GetCoord().TransformLocalToParent(pos);

    // Create the marker, attach it to the chassis body, add it to the list
    auto marker = chrono_types::make_shared<ChMarker>();
    marker->SetNameString(m_name + " " + name);
    marker->Impose_Rel_Coord(pos_com);
    m_body->AddMarker(marker);
    m_markers.push_back(marker);
}

void ChChassis::AddExternalForceTorque(std::shared_ptr<ExternalForceTorque> load) {
    m_loads.push_back(load);
    auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_body, ChVector<>(0), true, ChVector<>(0), true);
    m_external->Add(force_load);
    auto torque_load = chrono_types::make_shared<ChLoadBodyTorque>(m_body, ChVector<>(0), true);
    m_external->Add(torque_load);
}

void ChChassis::AddJoint(std::shared_ptr<ChVehicleJoint> joint) {
    if (joint->m_joint.index() == 0) {
        m_body->GetSystem()->AddLink(mpark::get<ChVehicleJoint::Link>(joint->m_joint));
    } else {
        m_bushings->Add(mpark::get<ChVehicleJoint::Bushing>(joint->m_joint));
    }
}

void ChChassis::RemoveJoint(std::shared_ptr<ChVehicleJoint> joint) {
    if (joint->m_joint.index() == 0) {
        ChVehicleJoint::Link& link = mpark::get<ChVehicleJoint::Link>(joint->m_joint);
        auto sys = link->GetSystem();
        if (sys) {
            sys->Remove(link);
        }
    }
    // Note: bushing are removed when the load container is removed
}

// -----------------------------------------------------------------------------

void ChChassis::InitializeInertiaProperties() {
    m_mass = GetBodyMass();
    m_inertia = GetBodyInertia();
    m_com = GetBodyCOMFrame();
}

void ChChassis::UpdateInertiaProperties() {
    m_xform = m_body->GetFrame_REF_to_abs();
}

// -----------------------------------------------------------------------------

// Chassis drag force implemented as an external force.
class ChassisDragForce : public ChChassis::ExternalForceTorque {
  public:
    ChassisDragForce(double Cd, double area, double air_density) : m_Cd(Cd), m_area(area), m_air_density(air_density) {}

    // The drag force, calculated based on the forward vehicle speed, is applied to
    // the center of mass of the chassis body.
    virtual void Update(double time,
                        const ChChassis& chassis,
                        ChVector<>& force,
                        ChVector<>& point,
                        ChVector<>& torque) override {
        auto body = chassis.GetBody();
        auto V = body->TransformDirectionParentToLocal(body->GetPos_dt());
        double Vx = V.x();
        double Fx = 0.5 * m_Cd * m_area * m_air_density * Vx * Vx;
        point = ChVector<>(0, 0, 0);
        force = ChVector<>(-Fx * ChSignum(Vx), 0.0, 0.0);
        torque = ChVector<>(0);
    }

  private:
    double m_Cd;           // drag coefficient
    double m_area;         // reference area (m2)
    double m_air_density;  // air density (kg/m3)
};

void ChChassis::SetAerodynamicDrag(double Cd, double area, double air_density) {
    auto drag_force = chrono_types::make_shared<ChassisDragForce>(Cd, area, air_density);
    AddExternalForceTorque(drag_force);
}

void ChChassis::Synchronize(double time) {
    // Update all external forces (two ChLoad objects per external force/torque)
    auto& loads = m_external->GetLoadList();
    ChVector<> force;
    ChVector<> point;
    ChVector<> torque;
    for (size_t i = 0; i < m_loads.size(); ++i) {
        m_loads[i]->Update(time, *this, force, point, torque);
        auto body_force = std::static_pointer_cast<ChLoadBodyForce>(loads[2 * i]);
        body_force->SetForce(force, true);
        body_force->SetApplicationPoint(point, true);
        auto body_torque = std::static_pointer_cast<ChLoadBodyTorque>(loads[2 * i + 1]);
        body_torque->SetTorque(torque, true);
    }
}

// -----------------------------------------------------------------------------

ChChassisRear::ChChassisRear(const std::string& name) : ChChassis(name, false) {}

void ChChassisRear::Initialize(std::shared_ptr<ChChassis> chassis, int collision_family) {
    // Express the rear chassis reference frame in the absolute coordinate system.
    // Set rear chassis orientation to be the same as the front chassis and
    // translate based on local positions of the connector point.
    const ChVector<>& front_loc = chassis->GetLocalPosRearConnector();
    const ChVector<>& rear_loc = GetLocalPosFrontConnector();

    ChFrame<> chassis_frame(front_loc - rear_loc);
    chassis_frame.ConcatenatePreTransformation(chassis->GetBody()->GetFrame_REF_to_abs());

    auto system = chassis->GetBody()->GetSystem();

    m_body = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
    m_body->SetIdentifier(0);
    m_body->SetNameString(m_name + " body");
    m_body->SetMass(GetBodyMass());
    m_body->SetFrame_COG_to_REF(GetBodyCOMFrame());
    m_body->SetInertia(GetBodyInertia());
    m_body->SetBodyFixed(m_fixed);

    m_body->SetFrame_REF_to_abs(chassis_frame);

    system->Add(m_body);

    // Add containers for bushing elements and external forces.
    system->Add(m_bushings);
    system->Add(m_external);

    // Add pre-defined marker (COM) on the chassis body.
    AddMarker("COM", ChCoordsys<>(GetBodyCOMFrame().GetPos(), GetBodyCOMFrame().GetRot()));

    // Mark as initialized
    m_initialized = true;
}

// -----------------------------------------------------------------------------

ChChassisConnector::ChChassisConnector(const std::string& name) : ChPart(name) {}

void ChChassisConnector::InitializeInertiaProperties() {
    m_mass = 0;
    m_inertia = ChMatrix33<>(0);
    m_com = ChFrame<>();
    m_xform = ChFrame<>();
}

void ChChassisConnector::UpdateInertiaProperties() {}

void ChChassisConnector::Initialize(std::shared_ptr<ChChassis> front, std::shared_ptr<ChChassisRear> rear) {
    // Mark as initialized
    m_initialized = true;
}

}  // end namespace vehicle
}  // end namespace chrono
