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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu
// =============================================================================
//
// Implementation of FSI system that includes all subclasses for proximity and
// force calculation, and time integration.
//
// =============================================================================

#ifndef CH_SYSTEM_FSI_H
#define CH_SYSTEM_FSI_H

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_fsi/physics/ChBce.cuh"
#include "chrono_fsi/physics/ChFluidDynamics.cuh"
#include "chrono_fsi/physics/ChFsiGeneral.h"
#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/ChFsiDefines.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"
#include "chrono_fsi/utils/ChUtilsJSON.h"

namespace chrono {

// Forward declarations
namespace fea {
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF_3423;
}

namespace fsi {

class ChSystemFsi_impl;

/// @addtogroup fsi_physics
/// @{

/// @brief Physical system for fluid-solid interaction problems.
///
/// This class is used to represent fluid-solid interaction problems consisting of fluid dynamics and multibody system.
/// Each of the two underlying physics is an independent object owned and instantiated by this class. Additionally, the
/// FSI system owns other objects to handle the interface between the two systems, boundary condition enforcing markers,
/// and data.
class CH_FSI_API ChSystemFsi {
  public:
    /// Constructor for FSI system.
    ChSystemFsi(ChSystem& other_physicalSystem,
                CHFSI_TIME_INTEGRATOR time_integrator = CHFSI_TIME_INTEGRATOR::ExplicitSPH);

    /// Destructor for the FSI system.
    ~ChSystemFsi();

    /// Function to integrate the FSI system in time.
    /// It uses a Runge-Kutta 2nd order algorithm to update both the fluid and multibody
    /// system dynamics. The midpoint data of MBS is needed for fluid dynamics update.
    void DoStepDynamics_FSI();

    /// Function to integrate the multibody system dynamics based on Runge-Kutta
    /// 2nd-order integration scheme.
    void DoStepDynamics_ChronoRK2();

    /// Function to initialize the midpoint device data of the fluid system by
    /// copying from the full step
    void CopyDeviceDataToHalfStep();

    /// Fill out the dependent data based on the independent one. For instance,
    /// it copies the position of the rigid bodies from the multibody dynamics system
    /// to arrays in FSI system since they are needed for internal use.
    void FinalizeData();

    /// Get a pointer to the data manager.
    std::shared_ptr<ChSystemFsi_impl> GetFsiData() { return fsiSystem; }

    /// Set the linear system solver for implicit methods
    void SetFluidSystemLinearSolver(ChFsiLinearSolver::SolverType other_solverType) {
        fluidDynamics->GetForceSystem()->SetLinearSolver(other_solverType);
    }

    /// Set the SPH method to be used for fluid dynamics.
    void SetFluidDynamics(fluid_dynamics params_type = fluid_dynamics::I2SPH);

    /// Get a pointer to the parameters used to set up the simulation.
    std::shared_ptr<SimParams> GetSimParams() { return paramsH; }

    /// Get a reference to the FSI bodies.
    /// FSI bodies are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<ChBody>>& GetFsiBodies() { return fsiBodies; }

    /// Get a reference to the FSI ChElementCableANCF.
    /// FSI ChElementCableANCF are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementCableANCF>>& GetFsiCables() { return fsiCables; }

    /// Get a reference to the FSI ChElementShellANCF_3423.
    /// FSI ChElementShellANCF_3423 are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& GetFsiShells() { return fsiShells; }

    /// Get a reference to the FSI ChNodeFEAxyzD.
    /// FSI ChNodeFEAxyzD are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& GetFsiNodes() { return fsiNodes; }

    /// Add FSI body to the FsiSystem.
    void AddFsiBody(std::shared_ptr<ChBody> mbody) { fsiBodies.push_back(mbody); }

    /// Complete construction of the FSI system (fluid and BDE objects).
    void Finalize();

    /// Finalize the construction of cable elements in the FSI system.
    void SetCableElementsNodes(std::vector<std::vector<int>> elementsNodes) {
        CableElementsNodes = elementsNodes;
        size_t test = fsiSystem->fsiGeneralData->CableElementsNodes.size();
        std::cout << "numObjects.numFlexNodes" << test << std::endl;
    }

    /// Finalize the construction of cable elements in the FSI system.
    void SetShellElementsNodes(std::vector<std::vector<int>> elementsNodes) {
        ShellElementsNodes = elementsNodes;
        size_t test = fsiSystem->fsiGeneralData->ShellElementsNodes.size();
        std::cout << "numObjects.numFlexNodes" << test << std::endl;
    }

    /// Set the FSI mesh for flexible elements.
    void SetFsiMesh(std::shared_ptr<fea::ChMesh> other_fsi_mesh) {
        fsi_mesh = other_fsi_mesh;
        fsiInterface->SetFsiMesh(other_fsi_mesh);
    }

    /// Set the FSI system output mode (default: NONE).
    void SetParticleOutputMode(CHFSI_OUTPUT_MODE mode) { file_write_mode = mode; }

    /// Write FSI system particle output.
    void WriteParticleFile(const std::string& outfilename) const;

    /// Save the SPH particle information into files.
    /// This function creates three files to write fluid, boundary, and BCE markers data to separate files.
    void PrintParticleToFile(const std::string& out_dir) const;

    /// Add SPH particle's information into the FSI system.
    void AddSphMarker(const ChVector<>& point,
                      double rho0,
                      double pres0,
                      double mu0,
                      double h,
                      double particle_type,
                      const ChVector<>& velocity = ChVector<>(0),
                      const ChVector<>& tauXxYyZz = ChVector<>(0),
                      const ChVector<>& tauXyXzYz = ChVector<>(0));

    /// Add reference array for SPH particles.
    void AddRefArray(const int start, const int numPart, const int compType, const int phaseType);

    /// Add BCE particle for a box.
    void AddBceBox(std::shared_ptr<SimParams> paramsH,
                   std::shared_ptr<ChBody> body,
                   const ChVector<>& relPos,
                   const ChQuaternion<>& relRot,
                   const ChVector<>& size,
                   int plane = 12,
                   bool isSolid = false);

    /// Add BCE particle for a sphere.
    void AddBceSphere(std::shared_ptr<SimParams> paramsH,
                      std::shared_ptr<ChBody> body,
                      const ChVector<>& relPos,
                      const ChQuaternion<>& relRot,
                      double radius);

    /// Add BCE particle for a cylinder.
    void AddBceCylinder(std::shared_ptr<SimParams> paramsH,
                        std::shared_ptr<ChBody> body,
                        const ChVector<>& relPos,
                        const ChQuaternion<>& relRot,
                        double radius,
                        double height,
                        double kernel_h,
                        bool cartesian = true);

    /// Add BCE particles from a set of points.
    void AddBceFromPoints(std::shared_ptr<SimParams> paramsH,
                          std::shared_ptr<ChBody> body,
                          const std::vector<ChVector<>>& points,
                          const ChVector<>& collisionShapeRelativePos,
                          const ChQuaternion<>& collisionShapeRelativeRot);

    /// Add BCE particle from a file.
    void AddBceFile(std::shared_ptr<SimParams> paramsH,
                    std::shared_ptr<ChBody> body,
                    const std::string& dataPath,
                    const ChVector<>& collisionShapeRelativePos,
                    const ChQuaternion<>& collisionShapeRelativeRot,
                    double scale,
                    bool isSolid = true);

    /// Add BCE particle from mesh.
    void AddBceFromMesh(std::shared_ptr<SimParams> paramsH,
                        std::shared_ptr<fea::ChMesh> my_mesh,
                        const std::vector<std::vector<int>>& NodeNeighborElement,
                        const std::vector<std::vector<int>>& _1D_elementsNodes,
                        const std::vector<std::vector<int>>& _2D_elementsNodes,
                        bool add1DElem,
                        bool add2DElem,
                        bool multiLayer,
                        bool removeMiddleLayer,
                        int SIDE,
                        int SIZE2D);

    /// Set FSI parameters from a JSON file.
    void SetSimParameter(const std::string& inputJson, std::shared_ptr<SimParams> paramsH, const ChVector<>& box_size);

    /// Set Periodic boundary condition for fluid.
    void SetBoundaries(const ChVector<>& cMin, const ChVector<>& cMax, std::shared_ptr<SimParams> paramsH);

    /// Set prescribed initial pressure for gravity field.
    void SetInitPressure(std::shared_ptr<SimParams> paramsH, const double fzDim);

    /// Gets the FSI mesh for flexible elements.
    std::shared_ptr<fea::ChMesh> GetFsiMesh() { return fsi_mesh; }

    /// Return the SPH kernel length of kernel function.
    float GetKernelLength() const;

    /// Set subdomains so that we find neighbor particles faster.
    void SetSubDomain(std::shared_ptr<SimParams> paramsH);

    /// Set output directory for FSI data.
    void SetFsiOutputDir(std::shared_ptr<SimParams> paramsH,
                         std::string& demo_dir,
                         std::string out_dir,
                         std::string inputJson);

    /// Return the SPH particle position.
    std::vector<ChVector<>> GetParticlePosOrProperties();

    /// Return the SPH particle velocity.
    std::vector<ChVector<>> GetParticleVel();

    /// Set SPH discretization type, consistent or inconsistent
    void SetDiscreType(bool useGmatrix, bool useLmatrix);

    /// Set FSI information output
    void SetFsiInfoOutput(bool outputFsiInfo);

    /// Set simulation data output length
    void SetOutputLength(int OutputLength);

    /// Set wall boundary condition
    void SetWallBC(BceVersion wallBC);

  private:
    /// Set the type of the fluid dynamics.
    void SetFluidIntegratorType(fluid_dynamics params_type);

    std::shared_ptr<ChSystemFsi_impl> fsiSystem;  ///< underlying system implementation

    CHFSI_OUTPUT_MODE file_write_mode;  ///< FSI particle output type (CSV, ChPF, or NONE)

    std::vector<std::shared_ptr<ChBody>> fsiBodies;                        ///< vector of a pointers to FSI bodies
    std::vector<std::shared_ptr<fea::ChElementCableANCF>> fsiCables;       ///< vector of cable ANCF elements
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>> fsiShells;  ///< vector of shell ANCF elements
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> fsiNodes;             ///< vector of FEA nodes
    std::shared_ptr<fea::ChMesh> fsi_mesh;                                 ///< FEA mesh

    std::vector<std::vector<int>> ShellElementsNodes;  ///< indices of nodes of each shell element
    std::vector<std::vector<int>> CableElementsNodes;  ///< indices of nodes of each cable element
    std::shared_ptr<ChFluidDynamics> fluidDynamics;    ///< pointer to the fluid system
    CHFSI_TIME_INTEGRATOR fluidIntegrator;             ///< IISPH by default
    std::shared_ptr<ChFsiInterface> fsiInterface;      ///< pointer to the FSI interface system
    std::shared_ptr<ChBce> bceWorker;                  ///< pointer to the bce workers
    std::shared_ptr<SimParams> paramsH;                ///< pointer to the simulation parameters
    std::shared_ptr<NumberOfObjects> numObjectsH;      ///< number of objects, fluid, bce, and boundary markers
    chrono::ChSystem& mphysicalSystem;                 ///< reference to the multi-body system

    double mTime;  ///< current real time of the simulation
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
