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
// Definition of the vehicle co-simulation flexible TIRE NODE class.
// This type of tire communicates with the terrain node through a MESH
// communication interface.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TIRE_NODE_FLEXIBLE_H
#define CH_VEHCOSIM_TIRE_NODE_FLEXIBLE_H

#include "chrono/fea/ChLoadContactSurfaceMesh.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/assets/ChVisualSystem.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChDeformableTire.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTireNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_tire
/// @{

/// Definition of the flexible tire node.
class CH_VEHICLE_API ChVehicleCosimTireNodeFlexible : public ChVehicleCosimTireNode {
  public:
    /// Construct an FEA tire co-simulation node with given (0-based) index from the given JSON specification file.
    /// The underlying Chrono system is set up to use a direct sparse linear solver (PardisoMKL if available) and the
    /// HHT integrator. By default, the solver will use 4 OpenMP threads for FEA analysis and 4 OpenMP threads for the
    /// linear solver.
    ChVehicleCosimTireNodeFlexible(int index, const std::string& tire_json);

    ~ChVehicleCosimTireNodeFlexible() {}

    /// Return the tire type.
    virtual TireType GetTireType() const override { return TireType::FLEXIBLE; }

    /// Attach FEA visual shape for run-time visualization.
    void AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shape);

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override final;

  private:
    /// A flexible tire implements the MESH communication interface.
    virtual InterfaceType GetInterfaceType() const override { return InterfaceType::MESH; }

    /// Initialize the tire by attaching it to the provided ChWheel.
    virtual void InitializeTire(std::shared_ptr<ChWheel> wheel, const ChVector3d& init_loc) override;

    /// Load current tire mesh state.
    virtual void LoadMeshState(MeshState& mesh_state) override;

    /// Return the tire force at the spindle.
    virtual void LoadSpindleForce(TerrainForce& spindle_force) override;

    /// Apply the spindle state (received from MBS node).
    virtual void ApplySpindleState(const BodyState& spindle_state) override;

    /// Apply the mesh contact forces (received from Terrain node).
    virtual void ApplyMeshForces(const MeshContact& mesh_contact) override;

    /// Perform additional output at the specified frame (called once per integration step).
    virtual void OnOutputData(int frame) override;

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override;

    /// Render tire system
    virtual void OnRender() override;

  private:
    /// Write mesh connectivity (and strain information)fixed).
    void WriteTireMeshInformation(utils::ChWriterCSV& csv);

    /// Write mesh vertex positions and velocities.
    void WriteTireStateInformation(utils::ChWriterCSV& csv);

    /// Write terrain forces applied to this tire.
    /// For a flexible tire, these are the forces on FEA mesh nodes.
    void WriteTireTerrainForces(utils::ChWriterCSV& csv);

    /// Print the current lowest mesh node.
    void PrintLowestNode();
    
    /// Print current contact forces.
    void PrintContactData(const std::vector<ChVector3d>& forces, const std::vector<int>& indices);

    std::shared_ptr<ChDeformableTire> m_tire_def;                   ///< deformable tire
    std::shared_ptr<fea::ChLoadContactSurfaceMesh> m_contact_load;  ///< tire contact surface
    std::vector<std::vector<unsigned int>> m_adjElements;  ///< list of neighboring elements for each mesh vertex
    std::vector<std::vector<unsigned int>> m_adjVertices;  ///< list of vertex indices for each mesh element
    MeshContact m_forces;                                  ///< cached nodal forces received from terrain node

    std::shared_ptr<ChVisualSystem> m_vsys;  ///< run-time visualization system
};

/// @} vehicle_cosim_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
