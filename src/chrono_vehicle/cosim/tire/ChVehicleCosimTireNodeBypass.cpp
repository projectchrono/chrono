// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2021 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Definition of a vehicle co-simulation TIRE NODE class which only
// intermediates communication between the MBS and Terrain nodes.
// This type of tire node communicates with the terrain node through a BODY
// communication interface.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono_vehicle/cosim/tire/ChVehicleCosimTireNodeBypass.h"

using std::cout;
using std::endl;

namespace chrono {
namespace vehicle {

ChVehicleCosimTireNodeBypass::ChVehicleCosimTireNodeBypass(int index, double mass, double radius, double width)
    : ChVehicleCosimTireNode(index), m_mass(mass), m_radius(radius), m_width(width) {
    // No mesh data
    m_mesh_data.nv = 1;
    m_mesh_data.nn = 1;
    m_mesh_data.nt = 1;

    // Set mesh data (vertex positions in local frame)
    m_mesh_data.verts.resize(1);
    m_mesh_data.norms.resize(1);
    m_mesh_data.idx_verts.resize(1);
    m_mesh_data.idx_norms.resize(1);

    // No tire contact material
    m_contact_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
}

}  // namespace vehicle
}  // namespace chrono
