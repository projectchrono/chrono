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

void ChRigidChassis::SetCollisionGeometry(const utils::ChBodyGeometry& geometry) {
    // Clear current collision objects
    if (m_geometry.HasCollision()) {
        m_geometry.materials.clear();
        m_geometry.coll_boxes.clear();
        m_geometry.coll_spheres.clear();
        m_geometry.coll_cylinders.clear();
        m_geometry.coll_cones.clear();
        m_geometry.coll_meshes.clear();
        m_geometry.coll_hulls.clear();
    }

    m_geometry.materials = geometry.materials;
    m_geometry.coll_boxes = geometry.coll_boxes;
    m_geometry.coll_spheres = geometry.coll_spheres;
    m_geometry.coll_cylinders = geometry.coll_cylinders;
    m_geometry.coll_cones = geometry.coll_cones;
    m_geometry.coll_meshes = geometry.coll_meshes;
    m_geometry.coll_hulls = geometry.coll_hulls;
}

void ChRigidChassis::OnInitialize(ChVehicle* vehicle, const ChCoordsys<>& chassisPos, double chassisFwdVel, int collision_family) {
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

void ChRigidChassisRear::OnInitialize(std::shared_ptr<ChChassis> chassis, int collision_family) {
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
