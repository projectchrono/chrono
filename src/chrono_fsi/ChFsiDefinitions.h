// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2024 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// Type definitions for Chrono::FSI (independent of a particular fluid solver)
//
// =============================================================================

#ifndef CH_FSI_DEFINITIONS_H
#define CH_FSI_DEFINITIONS_H

#include "chrono/ChConfig.h"

#include "chrono/physics/ChSystem.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceSegmentSet.h"
#include "chrono/utils/ChBodyGeometry.h"

namespace chrono {

using chrono::utils::ChBodyGeometry;

namespace fsi {

/// @addtogroup fsi_base
/// @{

/// Definition of a body state.
struct FsiBodyState {
    ChVector3d pos;      ///< global position
    ChQuaterniond rot;   ///< orientation with respect to global frame
    ChVector3d lin_vel;  ///< linear velocity, expressed in the global frame
    ChVector3d ang_vel;  ///< angular velocity, expressed in the global frame
    ChVector3d lin_acc;  ///< linear acceleration, expressed in the global frame
    ChVector3d ang_acc;  ///< angular acceleration, expressed in the global frame
};

/// Definition of a body wrench (force + torque).
struct FsiBodyForce {
    ChVector3d force;   ///< force at COM, expressed in the global frame
    ChVector3d torque;  ///< torque, expressed in the global frame
};

/// Definition of node states for a mesh.
struct FsiMeshState {
    std::vector<ChVector3d> pos;  ///< global positions
    std::vector<ChVector3d> vel;  ///< velocities, expressed in the global frame
    std::vector<ChVector3d> acc;  ///< accelerations, expressed in the global frame
    std::vector<ChVector3d> dir;  ///< node directions (unit vectors)

    bool has_node_directions;
};

/// Definition of a node forces for a mesh.
struct FsiMeshForce {
    std::vector<ChVector3d> force;  ///< force, expressed in the global frame
};

// -----------------------------------------------------------------------------

/// Methods for providing FEA node direction information.
/// If provided, node direction vectors can be used to provide a more accurate interpolation of positions between nodes
/// (piece-wise cubic as opposed to only piece-wise linear).
enum class NodeDirectionsMode {
    NONE,     ///< do not use FEA node directions (piece-wise linear interpolation)
    AVERAGE,  ///< approximate directions by averaging over incident elements
    EXACT     ///< use node directions communicated from FEA mesh
};

// -----------------------------------------------------------------------------

/// Description of a rigid body exposed to the FSI system.
struct FsiBody {
    size_t index;                              ///< body index in the list of FSI bodies
    std::shared_ptr<ChBody> body;              ///< rigid body exposed to FSI system
    std::shared_ptr<ChBodyGeometry> geometry;  ///< geometry for FSI interaction
    ChVector3d fsi_force;                      ///< fluid force at body COM (expressed in absolute frame)
    ChVector3d fsi_torque;                     ///< induced torque (expressed in absolute frame)
    unsigned int fsi_accumulator;              ///< index of the body force accumulator for fluid forces
};

/// Description of an FEA mesh surface with 1-D segments exposed to the FSI system.
struct FsiMesh1D {
    unsigned int GetNumElements() const { return contact_surface->GetNumSegments(); }
    unsigned int GetNumNodes() const { return (unsigned int)ind2ptr_map.size(); }

    size_t index;                                                      ///< mesh index in the list of FSI 1D meshes
    std::shared_ptr<fea::ChContactSurfaceSegmentSet> contact_surface;  ///< FEA contact segments
    std::map<std::shared_ptr<fea::ChNodeFEAxyz>, int> ptr2ind_map;     ///< pointer-based to index-based mapping
    std::map<int, std::shared_ptr<fea::ChNodeFEAxyz>> ind2ptr_map;     ///< index-based to pointer-based mapping
};

/// Description of an FEA mesh surface with 2-D faces exposed to the FSI system.
struct FsiMesh2D {
    unsigned int GetNumElements() const { return contact_surface->GetNumTriangles(); }
    unsigned int GetNumNodes() const { return (unsigned int)ind2ptr_map.size(); }

    size_t index;                                                   ///< mesh index in the list of FSI 2D meshes
    std::shared_ptr<fea::ChContactSurfaceMesh> contact_surface;     ///< FEA trimesh skin
    std::map<std::shared_ptr<fea::ChNodeFEAxyz>, int> ptr2ind_map;  ///< pointer-based to index-based mapping
    std::map<int, std::shared_ptr<fea::ChNodeFEAxyz>> ind2ptr_map;  ///< index-based to pointer-based mapping
};

/// @} fsi_base

}  // namespace fsi
}  // namespace chrono

#endif
