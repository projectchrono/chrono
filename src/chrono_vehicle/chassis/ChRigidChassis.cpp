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

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChVisualShapeSphere.h"
#include "chrono/assets/ChVisualShapeBox.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "chrono/utils/ChUtilsCreators.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/chassis/ChRigidChassis.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace vehicle {

// -----------------------------------------------------------------------------

ChRigidChassis::ChRigidChassis(const std::string& name, bool fixed) : ChChassis(name, fixed) {}

void ChRigidChassis::Construct(ChVehicle* vehicle,
                               const ChCoordsys<>& chassisPos,
                               double chassisFwdVel,
                               int collision_family) {
    // If collision shapes were defined, create the contact geometry and enable contact for the chassis's rigid body.
    // NOTE: setting the collision family is deferred to the containing vehicle system (which can also disable contact
    // between the chassis and certain vehicle subsystems).
    if (m_geometry.HasCollision()) {
        m_geometry.CreateCollisionShapes(m_body, collision_family, vehicle->GetSystem()->GetContactMethod());
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

// -----------------------------------------------------------------------------

ChRigidChassisRear::ChRigidChassisRear(const std::string& name) : ChChassisRear(name) {}

void ChRigidChassisRear::Construct(std::shared_ptr<ChChassis> chassis, int collision_family) {
    // If collision shapes were defined, create the contact geometry and enable contact for the chassis's rigid body.
    // NOTE: setting the collision family is deferred to the containing vehicle system (which can also disable contact
    // between the chassis and certain vehicle subsystems).
    if (m_geometry.HasCollision()) {
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

}  // end namespace vehicle
}  // end namespace chrono
