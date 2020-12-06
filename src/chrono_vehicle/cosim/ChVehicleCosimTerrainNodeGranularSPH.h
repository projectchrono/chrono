// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Wei Hu, Radu Serban
// =============================================================================
//
// Definition of the SPH granular TERRAIN NODE (using Chrono::FSI).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef TESTRIG_TERRAINNODE_GRANULAR_SPH_H
#define TESTRIG_TERRAINNODE_GRANULAR_SPH_H

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono_fsi/ChSystemFsi.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChVehicleCosimTerrainNodeGranularSPH : public ChVehicleCosimTerrainNode {
  public:
    /// Create a Chrono::FSI granular SPH terrain subsystem.
    ChVehicleCosimTerrainNodeGranularSPH(bool render);
    ~ChVehicleCosimTerrainNodeGranularSPH();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Specify the SPH terrain properties.
    void SetPropertiesSPH(const std::string& filename, double depth);
    /// Create SPH particles from an OBJ file
    void AddMeshMarkers(const std::string& mesh_filename,
                        double spearation,
                        std::vector<ChVector<>>& marker_positions);

  private:
    ChSystemSMC* m_system;          ///< containing system
    fsi::ChSystemFsi* m_systemFSI;  ///< containing FSI system

    std::shared_ptr<fsi::SimParams> m_params;  ///< FSI parameters
    double m_depth;                            ///< SPH soil depth

    virtual bool SupportsFlexibleTire() const override { return false; }

    virtual void Construct() override;

    virtual void CreateWheelProxy() override;
    virtual void UpdateWheelProxy() override;
    virtual void GetForceWheelProxy() override;
    virtual void PrintWheelProxyUpdateData() override;
    virtual void PrintWheelProxyContactData() override;

    virtual void OutputTerrainData(int frame) override;

    /// Advance simulation.
    /// This function is called after a synchronization to allow the node to advance
    /// its state by the specified time step.  A node is allowed to take as many internal
    /// integration steps as required, but no inter-node communication should occur.
    virtual void Advance(double step_size) override;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
