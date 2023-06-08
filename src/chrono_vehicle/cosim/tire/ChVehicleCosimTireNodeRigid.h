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
// Definition of the vehicle co-simulation rigid TIRE NODE class.
// This type of tire communicates with the terrain node through a BODY
// communication interface.
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM_TIRE_NODE_RIGID_H
#define CH_VEHCOSIM_TIRE_NODE_RIGID_H

#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTireNode.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_cosim_tire
/// @{

/// Definition of the rigid tire node.
class CH_VEHICLE_API ChVehicleCosimTireNodeRigid : public ChVehicleCosimTireNode {
  public:
    ChVehicleCosimTireNodeRigid(int index, const std::string& tire_json);
    ~ChVehicleCosimTireNodeRigid() {}

    /// Return the tire type.
    virtual TireType GetTireType() const override { return TireType::RIGID; }

    /// Advance simulation.
    /// A rigid tire node need not advance dynamics.
    virtual void Advance(double step_size) override final {}

  private:
    /// A rigid tire implements the BODY communication interface.
    virtual InterfaceType GetInterfaceType() const override { return InterfaceType::BODY; }

    /// Initialize the tire by attaching it to the provided ChWheel.
    virtual void InitializeTire(std::shared_ptr<ChWheel> wheel, const ChVector<>& init_loc) override;

    /// Apply the spindle state (received from MBS node).
    virtual void ApplySpindleState(const BodyState& spindle_state) override;

    /// Apply the spindle force (received from TERRAIN node).
    virtual void ApplySpindleForce(const TerrainForce& spindle_force) override;

    /// Perform additional output at the specified frame (called once per integration step).
    virtual void OnOutputData(int frame) override;

    /// Output post-processing visualization data.
    virtual void OutputVisualizationData(int frame) override;

  private:
    /// Write mesh vertex positions and velocities.
    void WriteTireStateInformation(utils::CSV_writer& csv);
    /// Write mesh connectivity and strain information.
    void WriteTireMeshInformation(utils::CSV_writer& csv);

    std::shared_ptr<ChRigidTire> m_tire_rgd;               ///< rigid tire
    std::vector<std::vector<unsigned int>> m_adjElements;  ///< list of neighboring elements for each mesh vertex
    std::vector<double> m_vertexArea;                      ///< representative areas for each mesh vertex
    TerrainForce m_force;                                  ///< cached force received from Terran node
};

/// @} vehicle_cosim_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
