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

#include "chrono_fsi/ChSystemFsi.h"

#include "chrono/physics/ChSystemSMC.h"

namespace chrono {
namespace vehicle {

class CH_VEHICLE_API ChVehicleCosimTerrainNodeGranularSPH : public ChVehicleCosimTerrainNode {
  
  public:
    /// Create a Chrono::FSI granular SPH terrain subsystem.
    ChVehicleCosimTerrainNodeGranularSPH(ChContactMethod method,  ///< contact method (penalty or complementatiry)
                                         bool render              ///< use OpenGL rendering
    );
    ~ChVehicleCosimTerrainNodeGranularSPH();

    virtual ChSystem* GetSystem() override { return m_system; }

    /// Specify the tire JSON specification file name.
    void SetTerrainJSONFile(const std::string& filename);

  private:

    ChSystemSMC* m_system;  ///< containing system 
    chrono::fsi::ChSystemFsi myFsiSystem(ChSystemSMC mphysicalSystem); ///< containing fsi system

    virtual bool SupportsFlexibleTire() const override { return false; }

    virtual void Construct() override;

    virtual void CreateWheelProxy() override;
    virtual void UpdateWheelProxy() override;
    virtual void GetForceWheelProxy() override;
    virtual void PrintWheelProxyUpdateData() override;
    virtual void PrintWheelProxyContactData() override;

    virtual void OutputTerrainData(int frame) override;
    virtual void OnSynchronize(int step_number, double time) override;
    virtual void OnAdvance(double step_size) override;

    // A fewer function in Chrono::FSI
    // void SetFluidDynamicsInFSI(paramsH->fluid_dynamic_type);
    // void SetFluidSystemLinearSolverInFSI(paramsH->LinearSolver);
    // void FinalizeDomainInFSI(paramsH);
    // void PrepareOutputDirInFSI(paramsH, demo_dir, out_dir, inputJson);
    // void CreateSPHparticlesInFSI(mphysicalSystem, myFsiSystem, paramsH);
    // void CreateSolidPhaseInFSI(mphysicalSystem, myFsiSystem, paramsH);
    // void FinalizeInFSI();
    void CreateSolidPhase(mphysicalSystem, myFsiSystem, paramsH);
  
  
};

}  // end namespace vehicle
}  // end namespace chrono

#endif
