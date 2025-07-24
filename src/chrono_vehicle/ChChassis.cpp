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

#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/utils/ChUtils.h"

#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/ChChassis.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChChassis::ChChassis(const std::string& name, bool fixed) : ChPart(name), m_fixed(fixed), m_vehicle(nullptr) {
    m_container_bushings = chrono_types::make_shared<ChLoadContainer>();
    m_container_external = chrono_types::make_shared<ChLoadContainer>();
    m_container_terrain = chrono_types::make_shared<ChLoadContainer>();
}

ChChassis::~ChChassis() {
    if (!m_initialized)
        return;

    auto sys = m_body->GetSystem();
    if (!sys)
        return;

    sys->Remove(m_body);
    sys->Remove(m_container_bushings);
    sys->Remove(m_container_external);
    sys->Remove(m_container_terrain);
}

// -----------------------------------------------------------------------------

uint16_t ChChassis::GetVehicleTag() const {
    ChAssertAlways(m_vehicle != nullptr);
    return m_vehicle->GetVehicleTag();
}

// -----------------------------------------------------------------------------

const ChVector3d& ChChassis::GetPos() const {
    return m_body->GetFrameRefToAbs().GetPos();
}

ChQuaternion<> ChChassis::GetRot() const {
    return m_body->GetFrameRefToAbs().GetRot() * ChWorldFrame::Quaternion();
}

ChVector3d ChChassis::GetPointLocation(const ChVector3d& locpos) const {
    return m_body->GetFrameRefToAbs().TransformPointLocalToParent(locpos);
}

ChVector3d ChChassis::GetPointVelocity(const ChVector3d& locpos) const {
    return m_body->GetFrameRefToAbs().PointSpeedLocalToParent(locpos);
}

ChVector3d ChChassis::GetPointAcceleration(const ChVector3d& locpos) const {
    ChVector3d acc_abs = m_body->GetFrameRefToAbs().PointAccelerationLocalToParent(locpos);
    return m_body->GetFrameRefToAbs().TransformDirectionParentToLocal(acc_abs);
}

// Return the global driver position
ChVector3d ChChassis::GetDriverPos() const {
    return m_body->GetFrameRefToAbs().TransformPointLocalToParent(GetLocalDriverCoordsys().pos);
}

// Return the speed measured at the origin of the chassis reference frame.
double ChChassis::GetSpeed() const {
    const auto& x_dir = m_body->GetRotMat().GetAxisX();
    const auto& vel = m_body->GetFrameRefToAbs().GetPosDt();
    return Vdot(vel, x_dir);
}

// Return the speed measured at the chassis center of mass.
double ChChassis::GetCOMSpeed() const {
    const auto& x_dir = m_body->GetRotMat().GetAxisX();
    const auto& vel = m_body->GetPosDt();
    return Vdot(vel, x_dir);
}

double ChChassis::GetRollRate() const {
    auto w = m_body->GetFrameRefToAbs().GetAngVelLocal();
    return w.x();
}

double ChChassis::GetPitchRate() const {
    auto w = m_body->GetFrameRefToAbs().GetAngVelLocal();
    return w.y();
}

double ChChassis::GetYawRate() const {
    auto w = m_body->GetFrameRefToAbs().GetAngVelLocal();
    return w.z();
}

double ChChassis::GetTurnRate() const {
    auto w = m_body->GetFrameRefToAbs().GetAngVelParent();
    return Vdot(w, ChWorldFrame::Vertical());
}

void ChChassis::Initialize(ChVehicle* vehicle,
                           const ChCoordsys<>& chassisPos,
                           double chassisFwdVel,
                           int collision_family) {
    ChAssertAlways(vehicle != nullptr);
    m_vehicle = vehicle;
    ChSystem* system = vehicle->GetSystem();

    // Set body tag
    m_obj_tag = VehicleObjTag::Generate(GetVehicleTag(), VehiclePartTag::CHASSIS);

    // Initial pose and velocity assumed to be given in current WorldFrame
    ChFrame<> chassis_pos(chassisPos.pos, ChMatrix33<>(chassisPos.rot) * ChWorldFrame::Rotation().transpose());

    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetTag(m_obj_tag);
    m_body->SetName(m_name + " body");
    m_body->SetMass(GetBodyMass());
    m_body->SetFrameCOMToRef(GetBodyCOMFrame());
    m_body->SetInertia(GetBodyInertia());
    m_body->SetFixed(m_fixed);

    m_body->SetFrameRefToAbs(chassis_pos);
    m_body->SetPosDt(chassisFwdVel * chassis_pos.TransformDirectionLocalToParent(ChVector3d(1, 0, 0)));

    system->Add(m_body);

    // Add containers for bushing elements and external forces.
    system->Add(m_container_bushings);
    system->Add(m_container_external);
    system->Add(m_container_terrain);

    // Add pre-defined markers (driver position and COM) on the chassis body.
    AddMarker("driver position", ChFrame<>(GetLocalDriverCoordsys()));
    AddMarker("COM", GetCOMFrame());

    Construct(vehicle, chassisPos, chassisFwdVel, collision_family);

    // Mark as initialized
    m_initialized = true;
}

void ChChassis::AddMarker(const std::string& name, const ChFrame<>& frame) {
    // Do nothing if the chassis is not yet initialized
    if (!m_body)
        return;

    // Note: marker local positions are assumed to be relative to the centroidal frame
    //       of the associated body.
    ChFrame<> frame_com = m_body->GetFrameRefToCOM().TransformLocalToParent(frame);

    // Create the marker, attach it to the chassis body, add it to the list
    auto marker = chrono_types::make_shared<ChMarker>();
    marker->SetName(m_name + " " + name);
    marker->ImposeRelativeTransform(frame_com);
    m_body->AddMarker(marker);
    m_markers.push_back(marker);
}

void ChChassis::AddExternalForceTorque(std::shared_ptr<ExternalForceTorque> load) {
    m_external_loads.push_back(load);

    auto force_load = chrono_types::make_shared<ChLoadBodyForce>(m_body, ChVector3d(0), true, ChVector3d(0), true);
    force_load->SetName(load->m_name + "_force");
    m_container_external->Add(force_load);

    auto torque_load = chrono_types::make_shared<ChLoadBodyTorque>(m_body, ChVector3d(0), true);
    torque_load->SetName(load->m_name + "_torque");
    m_container_external->Add(torque_load);
}

void ChChassis::AddJoint(std::shared_ptr<ChJoint> joint) {
    if (joint->IsKinematic()) {
        m_body->GetSystem()->AddLink(joint->GetAsLink());
    } else {
        m_container_bushings->Add(joint->GetAsBushing());
    }
}

void ChChassis::RemoveJoint(std::shared_ptr<ChJoint> joint) {
    if (joint->IsKinematic()) {
        auto link = joint->GetAsLink();
        auto sys = link->GetSystem();
        if (sys) {
            sys->Remove(link);
        }
    }
    // Note: bushing are removed when the load container is removed
}

void ChChassis::AddTerrainLoad(std::shared_ptr<ChLoadBase> terrain_load) {
    m_container_terrain->Add(terrain_load);
}

// -----------------------------------------------------------------------------

void ChChassis::InitializeInertiaProperties() {
    m_mass = GetBodyMass();
    m_inertia = GetBodyInertia();
    m_com = GetBodyCOMFrame();
}

void ChChassis::UpdateInertiaProperties() {
    m_xform = m_body->GetFrameRefToAbs();
}

// -----------------------------------------------------------------------------

// Chassis drag force implemented as an external force.
class ChassisDragForce : public ChChassis::ExternalForceTorque {
  public:
    ChassisDragForce(double Cd, double area, double air_density)
        : ExternalForceTorque("Chassis_drag"), m_Cd(Cd), m_area(area), m_air_density(air_density) {}

    // The drag force, calculated based on the forward vehicle speed, is applied to
    // the center of mass of the chassis body.
    virtual void Update(double time,
                        const ChChassis& chassis,
                        ChVector3d& force,
                        ChVector3d& point,
                        ChVector3d& torque) override {
        auto body = chassis.GetBody();
        auto V = body->TransformDirectionParentToLocal(body->GetPosDt());
        double Vx = V.x();
        double Fx = 0.5 * m_Cd * m_area * m_air_density * Vx * Vx;
        point = ChVector3d(0, 0, 0);
        force = ChVector3d(-Fx * ChSignum(Vx), 0.0, 0.0);
        torque = ChVector3d(0);
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
    auto& loads = m_container_external->GetLoadList();
    ChVector3d force;
    ChVector3d point;
    ChVector3d torque;
    for (size_t i = 0; i < m_external_loads.size(); ++i) {
        m_external_loads[i]->Update(time, *this, force, point, torque);
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
    m_vehicle = chassis->m_vehicle;

    m_parent = chassis;
    m_obj_tag = VehicleObjTag::Generate(GetVehicleTag(), VehiclePartTag::CHASSIS_REAR);

    // Express the rear chassis reference frame in the absolute coordinate system.
    // Set rear chassis orientation to be the same as the front chassis and
    // translate based on local positions of the connector point.
    const ChVector3d& front_loc = chassis->GetLocalPosRearConnector();
    const ChVector3d& rear_loc = GetLocalPosFrontConnector();

    ChFrame<> chassis_frame(front_loc - rear_loc);
    chassis_frame.ConcatenatePreTransformation(chassis->GetBody()->GetFrameRefToAbs());

    auto system = chassis->GetBody()->GetSystem();

    m_body = chrono_types::make_shared<ChBodyAuxRef>();
    m_body->SetTag(m_obj_tag);
    m_body->SetName(m_name + " body");
    m_body->SetMass(GetBodyMass());
    m_body->SetFrameCOMToRef(GetBodyCOMFrame());
    m_body->SetInertia(GetBodyInertia());
    m_body->SetFixed(m_fixed);

    m_body->SetFrameRefToAbs(chassis_frame);

    system->Add(m_body);

    // Add containers for bushing elements and external forces.
    system->Add(m_container_bushings);
    system->Add(m_container_external);
    system->Add(m_container_terrain);

    // Add pre-defined marker (COM) on the chassis body.
    AddMarker("COM", GetBodyCOMFrame());

    Construct(chassis, collision_family);

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
