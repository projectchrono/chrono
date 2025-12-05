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
// Chassis subsystem for the generic vehicle.
//
// =============================================================================

#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/input_output/ChWriterCSV.h"

#include "chrono_vehicle/ChVehicleDataPath.h"

#include "chrono_models/vehicle/generic/Generic_Chassis.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables

const double Generic_Chassis::m_body_mass = 2086.52;
const ChVector3d Generic_Chassis::m_body_inertiaXX(1078.52, 2955.66, 3570.20);
const ChVector3d Generic_Chassis::m_body_inertiaXY(0, 0, 0);
const ChVector3d Generic_Chassis::m_body_COM_loc(0.056, 0, 0.213);
const ChVector3d Generic_Chassis::m_connector_rear_loc(-2.5, 0, -0.25);
const ChCoordsys<> Generic_Chassis::m_driverCsys(ChVector3d(0.87, 0.7, 1.05), ChQuaternion<>(1, 0, 0, 0));

// -----------------------------------------------------------------------------

Generic_Chassis::Generic_Chassis(const std::string& name, bool fixed, CollisionType chassis_collision_type)
    : ChRigidChassis(name, fixed) {
    // In this model, we use a single contact material.
    ChContactMaterialData minfo;
    minfo.mu = 1.0f;
    minfo.cr = 0.1f;
    minfo.Y = 5e5f;
    m_geometry.materials.push_back(minfo);

    m_body_inertia(0, 0) = m_body_inertiaXX.x();
    m_body_inertia(1, 1) = m_body_inertiaXX.y();
    m_body_inertia(2, 2) = m_body_inertiaXX.z();

    m_body_inertia(0, 1) = m_body_inertiaXY.x();
    m_body_inertia(0, 2) = m_body_inertiaXY.y();
    m_body_inertia(1, 2) = m_body_inertiaXY.z();
    m_body_inertia(1, 0) = m_body_inertiaXY.x();
    m_body_inertia(2, 0) = m_body_inertiaXY.y();
    m_body_inertia(2, 1) = m_body_inertiaXY.z();

    //// TODO:
    //// A more appropriate contact shape from primitives
    utils::ChBodyGeometry::BoxShape box1(ChVector3d(0.0, 0.0, 0.1), ChQuaternion<>(1, 0, 0, 0),
                                         ChVector3d(2.0, 1.0, 0.2));
    utils::ChBodyGeometry::BoxShape box2(ChVector3d(0.0, 0.0, 0.3), ChQuaternion<>(1, 0, 0, 0),
                                         ChVector3d(1.0, 0.5, 0.2));

    m_geometry.vis_boxes.push_back(box1);
    m_geometry.vis_boxes.push_back(box2);

    switch (chassis_collision_type) {
        case CollisionType::PRIMITIVES:
            box1.matID = 0;
            m_geometry.coll_boxes.push_back(box1);
            break;
        default:
            break;
    }
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
