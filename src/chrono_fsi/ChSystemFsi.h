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
// Author: Milad Rakhsha, Arman Pazouki
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration.
//
// =============================================================================

#ifndef CH_SYSTEMFSI_H_
#define CH_SYSTEMFSI_H_

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/physics/ChFluidDynamics.cuh"
#include "chrono_fsi/ChFsiDataManager.cuh"
#include "chrono_fsi/physics/ChFsiGeneral.cuh"
#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

namespace chrono {

// Forward declaration
namespace fea {
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF;
}  // namespace fea

/// Output mode of system.
enum class CHFSI_OUTPUT_MODE { CSV, CHPF, NONE };

namespace fsi {

/// @addtogroup fsi_physics
/// @{

/// @brief Physical system for fluid-solid interaction problem.
///
/// This class is used to represent a fluid-solid interaction problem consist of
/// fluid dynamics and multibody system. Each of the two underlying physics are
/// independent objects owned and instantiated by this class. Additionally,
/// the fsi system owns other objects to handle the interface between the two
/// systems, boundary condition enforcing markers, and data.
class CH_FSI_API ChSystemFsi : public ChFsiGeneral {
  public:
    /// Constructor for FSI system.
    /// This class constructor instantiates all the member objects. Wherever relevant, the
    /// instantiation is handled by sending a pointer to other objects or data.
    /// Therefore, the sub-classes have pointers to the same data.
    ChSystemFsi(ChSystem& other_physicalSystem, ChFluidDynamics::Integrator type = ChFluidDynamics::Integrator::IISPH);

    /// Destructor for the FSI system.
    ~ChSystemFsi();

    /// Function to integrate the fsi system in time.
    /// It uses a Runge-Kutta 2nd order algorithm to update both the fluid and multibody
    /// system dynamics. The midpoint data of MBS is needed for fluid dynamics update.
    virtual void DoStepDynamics_FSI();

    /// Function to integrate the multibody system dynamics based on Runge-Kutta
    /// 2nd-order integration scheme.
    virtual void DoStepDynamics_ChronoRK2();

    /// Function to initialize the midpoint device data of the fluid system by
    /// copying from the full step
    virtual void CopyDeviceDataToHalfStep();

    /// Fill out the dependent data based on the independent one. For instance,
    /// it copies the position of the rigid bodies from the multibody dynamics system
    /// to arrays in fsi system since they are needed for internal use.
    virtual void FinalizeData();

    /// Get a pointer to the data manager.
    std::shared_ptr<ChFsiDataManager> GetDataManager() { return fsiData; }

    /// Set the linear system solver for implicit methods
    void SetFluidSystemLinearSolver(ChFsiLinearSolver::SolverType other_solverType) {
        fluidDynamics->GetForceSystem()->SetLinearSolver(other_solverType);
    }

    /// Set the SPH method to be used for fluid dynamics
    void SetFluidDynamics(fluid_dynamics params_type = fluid_dynamics::I2SPH);

    /// Get a pointer to the parameters used to set up the simulation.
    std::shared_ptr<SimParams> GetSimParams() { return paramsH; }

    /// Get a reference to the fsi bodies.
    /// fsi bodies are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<ChBody>>& GetFsiBodies() { return fsiBodeis; }

    /// Get a reference to the fsi ChElementCableANCF.
    /// fsi ChElementCableANCF are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementCableANCF>>& GetFsiCables() { return fsiCables; }

    /// Get a reference to the fsi ChElementShellANCF.
    /// fsi ChElementShellANCF are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementShellANCF>>& GetFsiShells() { return fsiShells; }

    /// Get a reference to the fsi ChNodeFEAxyzD.
    /// fsi ChNodeFEAxyzD are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& GetFsiNodes() { return fsiNodes; }

    /// Adds FSI body to the FsiSystem
    void AddFsiBody(std::shared_ptr<ChBody> mbody) { fsiBodeis.push_back(mbody); }

    /// Finzalizes data by calling FinalizeData function and finalize fluid and bce
    /// and also finalizes the fluid and bce objects if the system have fluid.
    virtual void Finalize();

    /// Finalizes the construction of cable elements in the FSI system.
    void SetCableElementsNodes(std::vector<std::vector<int>> elementsNodes) {
        CableElementsNodes = elementsNodes;
        size_t test = fsiData->fsiGeneralData->CableElementsNodes.size();
        std::cout << "numObjects.numFlexNodes" << test << std::endl;
    }

    /// Finalizes the construction of cable elements in the FSI system.
    void SetShellElementsNodes(std::vector<std::vector<int>> elementsNodes) {
        ShellElementsNodes = elementsNodes;
        size_t test = fsiData->fsiGeneralData->ShellElementsNodes.size();
        std::cout << "numObjects.numFlexNodes" << test << std::endl;
    }

    /// Sets the FSI mesh for flexible elements.
    void SetFsiMesh(std::shared_ptr<fea::ChMesh> other_fsi_mesh) {
        fsi_mesh = other_fsi_mesh;
        fsiInterface->SetFsiMesh(other_fsi_mesh);
    }

    /// Sets the FSI system output mode.
    void SetParticleOutputMode(CHFSI_OUTPUT_MODE mode) { file_write_mode = mode; }

    /// Write FSI system particle output
    void WriteParticleFile(const std::string& outfilename) const;

    /// Gets the FSI mesh for flexible elements.
    std::shared_ptr<fea::ChMesh> GetFsiMesh() { return fsi_mesh; }

  private:
    /// Integrate the chrono system based on an explicit Euler scheme.
    int DoStepChronoSystem(Real dT, double mTime);
    /// Set the type of the fluid dynamics
    void SetFluidIntegratorType(fluid_dynamics params_type);

    CHFSI_OUTPUT_MODE file_write_mode;  ///< FSI particle output type::CSV | ChPF | None, default is NONE

    std::shared_ptr<ChFsiDataManager> fsiData;       ///< Pointer to data manager which holds all the data
    std::vector<std::shared_ptr<ChBody>> fsiBodeis;  ///< Vector of a pointers to fsi bodies. fsi bodies
                                                     /// are those that interact with fluid
    std::vector<std::shared_ptr<fea::ChElementCableANCF>> fsiCables;  ///< Vector of ChElementShellANCF pointers
    std::vector<std::shared_ptr<fea::ChElementShellANCF>> fsiShells;  ///< Vector of ChElementShellANCF pointers
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> fsiNodes;        ///< Vector of ChNodeFEAxyzD nodes
    std::shared_ptr<fea::ChMesh> fsi_mesh;                            ///< ChMesh Pointer

    std::vector<std::vector<int>> ShellElementsNodes;  ///< Indices of nodes of each Element
    std::vector<std::vector<int>> CableElementsNodes;  ///< Indices of nodes of each Element
    std::shared_ptr<ChFluidDynamics> fluidDynamics;    ///< pointer to the fluid system
    ChFluidDynamics::Integrator fluidIntegrator;       ///< IISPH by default
    std::shared_ptr<ChFsiInterface> fsiInterface;      ///< pointer to the fsi interface system
    std::shared_ptr<ChBce> bceWorker;                  ///< pointer to the bce workers
    std::shared_ptr<SimParams> paramsH;                ///< pointer to the simulation parameters
    std::shared_ptr<NumberOfObjects> numObjectsH;      ///< number of objects, fluid, bce, and boundary markers
    chrono::ChSystem& mphysicalSystem;                 ///< Reference to the multi-body system

    double mTime;  ///< current real time of the simulation
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
