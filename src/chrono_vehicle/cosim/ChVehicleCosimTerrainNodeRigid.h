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
// Authors: Radu Serban
// =============================================================================
//
// Definition of the rigid TERRAIN NODE (using Chrono).
//
// The global reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#ifndef CH_VEHCOSIM__TERRAINNODE_RIGID_H
#define CH_VEHCOSIM__TERRAINNODE_RIGID_H

#include <vector>

#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"

#include "chrono_vehicle/cosim/ChVehicleCosimTerrainNode.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChVehicleCosimTerrainNodeRigid : public ChVehicleCosimTerrainNode {
  public:
    /// Create a rigid terrain subsystem.
    ChVehicleCosimTerrainNodeRigid(ChContactMethod method,  ///< contact method (penalty or complementatiry)
                                   bool render              ///< use OpenGL rendering
    );
    ~ChVehicleCosimTerrainNodeRigid();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Set the material properties for terrain.
    /// The type of material must be consistent with the contact method (penalty or complementarity)
    /// specified at construction. These parameters characterize the material for the container and
    /// (if applicable) the granular material.  Tire contact material is received from the rig node.
    virtual void SetMaterialSurface(const std::shared_ptr<ChMaterialSurface>& mat) override;

    /// Specify whether contact coefficients are based on material properties (default: true).
    /// Note that this setting is only relevant when using the penalty method.
    virtual void UseMaterialProperties(bool flag) override;

    /// Set the normal contact force model (default: Hertz)
    /// Note that this setting is only relevant when using the penalty method.
    virtual void SetContactForceModel(ChSystemSMC::ContactForceModel model) override;

    /// Obtain settled terrain configuration.
    /// No-op for a rigid terrain.
    virtual void Settle() override {}

  private:
    ChSystem* m_system;  ///< containing system

    virtual void Construct() override;
    virtual void CreateProxies() override;
    virtual void UpdateProxies() override;
    virtual void ForcesProxies(std::vector<double>& vert_forces, std::vector<int>& vert_indices) override;
    virtual void PrintProxiesUpdateData() override;
    virtual void PrintProxiesContactData() override;
    virtual void OutputTerrainData(int frame) override;
    virtual void OnSynchronize(int step_number, double time) override;
    virtual void OnAdvance(double step_size) override;
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
