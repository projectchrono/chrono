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
// Template for a rigid-body chassis vehicle subsystem.
//
// =============================================================================

#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChCylinderShape.h"

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChRigidChassis::ChRigidChassis(const std::string& name, bool fixed) : ChChassis(name, fixed) {}

void ChRigidChassis::Initialize(ChSystem* system,
                                const ChCoordsys<>& chassisPos,
                                double chassisFwdVel,
                                int collision_family) {
    // Invoke the base class method to construct the frame body.
    ChChassis::Initialize(system, chassisPos, chassisFwdVel);

    // If collision shapes were defined, create the contact geometry and enable contact
    // for the chassis's rigid body.
    // NOTE: setting the collision family is deferred to the containing vehicle system
    // (which can also disable contact between the chassis and certain vehicle subsystems).
    if (m_geometry.m_has_collision) {
        m_geometry.CreateCollisionShapes(m_body, collision_family, system->GetContactMethod());
    }
}

void ChRigidChassis::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_geometry.CreateVisualizationAssets(m_body, vis);
}

void ChRigidChassis::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_body);
}

void ChRigidChassis::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_body);
    ChPart::ExportBodyList(jsonDocument, bodies);

    ChPart::ExportMarkerList(jsonDocument, m_markers);
}

void ChRigidChassis::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBodyAuxRef>> bodies;
    bodies.push_back(m_body);
    database.WriteAuxRefBodies(bodies);

    database.WriteMarkers(m_markers);
}

// -----------------------------------------------------------------------------

ChRigidChassisRear::ChRigidChassisRear(const std::string& name) : ChChassisRear(name) {}

void ChRigidChassisRear::Initialize(std::shared_ptr<ChChassis> chassis,
                                    int collision_family) {
    // Invoke the base class method to construct the frame body.
    ChChassisRear::Initialize(chassis, collision_family);

    // If collision shapes were defined, create the contact geometry and enable contact
    // for the chassis's rigid body.
    // NOTE: setting the collision family is deferred to the containing vehicle system
    // (which can also disable contact between the chassis and certain vehicle subsystems).
    if (m_geometry.m_has_collision) {
        m_geometry.CreateCollisionShapes(m_body, collision_family, m_body->GetSystem()->GetContactMethod());
    }
}

void ChRigidChassisRear::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::NONE)
        return;

    m_geometry.CreateVisualizationAssets(m_body, vis);
}

void ChRigidChassisRear::RemoveVisualizationAssets() {
    ChPart::RemoveVisualizationAssets(m_body);
}

void ChRigidChassisRear::ExportComponentList(rapidjson::Document& jsonDocument) const {
    ChPart::ExportComponentList(jsonDocument);

    std::vector<std::shared_ptr<ChBody>> bodies;
    bodies.push_back(m_body);
    ChPart::ExportBodyList(jsonDocument, bodies);

    ChPart::ExportMarkerList(jsonDocument, m_markers);
}

void ChRigidChassisRear::Output(ChVehicleOutput& database) const {
    if (!m_output)
        return;

    std::vector<std::shared_ptr<ChBodyAuxRef>> bodies;
    bodies.push_back(m_body);
    database.WriteAuxRefBodies(bodies);

    database.WriteMarkers(m_markers);
}

}  // end namespace vehicle
}  // end namespace chrono
