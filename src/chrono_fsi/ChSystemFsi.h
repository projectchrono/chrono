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
// Author: Milad Rakhsha, Arman Pazouki, Wei Hu, Radu Serban
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
#include "chrono_fsi/ChFsiInterface.h"
#include "chrono_fsi/utils/ChUtilsPrintSph.cuh"

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
    /// Output mode.
    enum class OutpuMode {
        CSV,   ///< comma-separated value
        CHPF,  ///< binary
        NONE   ///< none
    };

    /// Structure with elastic material properties.
    /// Used if solving an SPH continuum representation of granular dynamics.
    struct ElasticMaterialProperties {
        double Young_modulus;     ///< Young's modulus
        double Poisson_ratio;     ///< Poisson�s ratio
        double stress;            ///< Artifical stress
        double viscosity_alpha;   ///< Artifical viscosity coefficient
        double viscosity_beta;    ///< Artifical viscosity coefficient
        double mu_I0;             ///< Reference Inertia number
        double mu_fric_s;         ///< friction mu_s
        double mu_fric_2;         ///< mu_2 constant in mu=mu(I)
        double average_diam;      ///< average particle diameter
        double friction_angle;    ///< Frictional angle of granular material
        double dilation_angle;    ///< Dilate angle of granular material
        double cohesion_coeff;    ///< Cohesion coefficient
        double kernel_threshold;  ///< Threshold of the integration of the kernel function

        ElasticMaterialProperties();
    };

    /// Constructor for FSI system.
    ChSystemFsi(ChSystem& other_physicalSystem);

    /// Destructor for the FSI system.
    ~ChSystemFsi();

    /// Function to integrate the FSI system in time.
    /// It uses a Runge-Kutta 2nd order algorithm to update both the fluid and multibody system dynamics. The midpoint
    /// data of MBS is needed for fluid dynamics update.
    void DoStepDynamics_FSI();

    /// Function to integrate the multibody system dynamics based on Runge-Kutta 2nd-order integration scheme.
    void DoStepDynamics_ChronoRK2();

    /// Enable/disable verbose terminal output.
    void SetVerbose(bool verbose);

    /// Read Chrono::FSI parameters from the specified JSON file.
    void ReadParametersFromFile(const std::string& json_file);

    /// Set initial spacing.
    void SetInitialSpacing(double spacing);

    /// Set SPH kernel length.
    void SetKernelLength(double length);

    /// Set the fluid container dimension
    void SetContainerDim(const ChVector<>& boxDim);

    /// Set periodic boundary condition for fluid.
    void SetBoundaries(const ChVector<>& cMin, const ChVector<>& cMax);

    /// Set number of boundary layers (default: 3).
    void SetNumBoundaryLayers(int num_layers);

    /// Set (initial) density.
    void SetDensity(double rho0);

    /// Set prescribed initial pressure for gravity field.
    void SetInitPressure(const double fzDim);

    /// Set gravity for the FSI syatem.
    void Set_G_acc(const ChVector<>& gravity);

    /// Set a constant force applied to the fluid.
    /// Solid bodies are not explicitly affected by this force, but they are affected indirectly through the fluid.
    void SetBodyForce(const ChVector<>& force);

    /// Set FSI integration step size.
    void SetStepSize(double dT, double dT_Flex = 0);

    /// Set the maximum allowable integration step size.
    void SetMaxStepSize(double dT_max);

    /// Enable/disable adaptive time stepping.
    void SetAdaptiveTimeStepping(bool adaptive);

    /// Set SPH discretization type, consistent or inconsistent
    void SetDiscreType(bool useGmatrix, bool useLmatrix);

    /// Set wall boundary condition
    void SetWallBC(BceVersion wallBC);

    /// Set the linear system solver for implicit methods.
    void SetSPHLinearSolver(ChFsiLinearSolver::SolverType lin_solver);

    /// Set the SPH method and, optionally, the linear solver type.
    void SetSPHMethod(FluidDynamics SPH_method,
                      ChFsiLinearSolver::SolverType lin_solver = ChFsiLinearSolver::SolverType::BICGSTAB);

    /// Enable solution of elastic SPH (for continuum representation of granular dynamics).
    /// By default, a ChSystemFSI solves an SPH fluid dynamics problem.
    void SetElasticSPH(const ElasticMaterialProperties mat_props);

    /// Set output directory for FSI data.
    /// No output is generated by default (for better performance). If an output directory is specified, output files
    /// with information on each FSI body and node are generated in a subdirectory 'fsi/' of the given output directory.
    /// It is the caller's responsibility to ensure that output_dir exists.
    void SetOutputDirectory(const std::string& output_dir);

    /// Set simulation data output length
    void SetOutputLength(int OutputLength);

    /// Set the FSI system output mode (default: NONE).
    void SetParticleOutputMode(OutpuMode mode) { file_write_mode = mode; }

    /// Return the SPH kernel length of kernel function.
    double GetKernelLength() const;

    /// Return the initial spacing of the SPH particles.
    double GetInitialSpacing() const;

    /// Set the fluid container dimension
    ChVector<> GetContainerDim() const;

    /// Return density.
    double GetDensity() const;

    /// Return viscosity.
    double GetViscosity() const;

    /// Return SPH particle mass.
    double GetParticleMass() const;

    /// Return base pressure.
    double GetBasePressure() const;

    /// Return gravitational acceleration.
    ChVector<> Get_G_acc() const;

    /// Return the speed of sound in the fluid phase.
    double GetSoundSpeed() const;

    /// Return the constant force applied to the fluid (if any).
    ChVector<> GetBodyForce() const;

    /// Return the FSI integration step size.
    double GetStepSize() const;

    /// Return the current value of the maximum allowable integration step size.
    double GetMaxStepSize() const;

    /// Return a flag inicating whether adaptive time stepping is enabled.
    bool GetAdaptiveTimeStepping() const;

    /// Get the current number of fluid SPH particles.
    size_t GetNumFluidMarkers() const;

    /// Get the current number of boundary BCE markers.
    size_t GetNumBoundaryMarkers() const;

    /// Get the current number of rigid body BCE markers.
    size_t GetNumRigidBodyMarkers() const;

    /// Get the current number of flexible body BCE markers.
    size_t GetNumFlexBodyMarkers() const;

    /// Get current simulation time.
    double GetSimTime() const { return mTime; }

    /// Return the SPH particle position.
    std::vector<ChVector<>> GetParticlePosOrProperties();

    /// Return the SPH particle velocity.
    std::vector<ChVector<>> GetParticleVel();

    /// Get a reference to the FSI bodies.
    /// FSI bodies are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<ChBody>>& GetFsiBodies() { return fsiBodies; }

    /// Return the FSI mesh for flexible elements.
    std::shared_ptr<fea::ChMesh> GetFsiMesh() { return fsi_mesh; }

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

    /// Set number of nodes in FEA cable elements in the FSI system.
    void SetCableElementsNodes(std::vector<std::vector<int>> elementsNodes);

    /// Set number of nodes in FEA shell elements in the FSI system.
    void SetShellElementsNodes(std::vector<std::vector<int>> elementsNodes);

    /// Set the FSI mesh for flexible elements.
    void SetFsiMesh(std::shared_ptr<fea::ChMesh> other_fsi_mesh);

    /// Complete construction of the FSI system (fluid and BDE objects).
    /// Use parameters read from JSON file and/or specified through various Set functions.
    void Initialize();

    /// Write FSI system particle output.
    void WriteParticleFile(const std::string& outfilename) const;

    /// Save the SPH particle information into files.
    /// This function creates three files to write fluid, boundary, and BCE markers data to separate files.
    void PrintParticleToFile(const std::string& dir) const;

    /// Add an SPH particle with given properties to the FSI system.
    void AddSphMarker(const ChVector<>& point,
                      double rho0,
                      double pres0,
                      double mu0,
                      double h,
                      double particle_type,
                      const ChVector<>& velocity = ChVector<>(0),
                      const ChVector<>& tauXxYyZz = ChVector<>(0),
                      const ChVector<>& tauXyXzYz = ChVector<>(0));

    /// Add an SPH particle with current properties to the SPH system.
    void AddSphMarker(const ChVector<>& point,
                      double particle_type,
                      const ChVector<>& velocity = ChVector<>(0),
                      const ChVector<>& tauXxYyZz = ChVector<>(0),
                      const ChVector<>& tauXyXzYz = ChVector<>(0));

    /// Add reference array for SPH particles.
    void AddRefArray(const int start, const int numPart, const int compType, const int phaseType);

    /// Create SPH particle from a box domain
    void AddSphMarkerBox(double initSpace,
                         double kernelLength,
                         const ChVector<>& boxCenter,
                         const ChVector<>& boxHalfDim);

    /// Add BCE particle for a box.
    void AddBceBox(std::shared_ptr<ChBody> body,
                   const ChVector<>& relPos,
                   const ChQuaternion<>& relRot,
                   const ChVector<>& size,
                   int plane = 12,
                   bool isSolid = false);

    /// Add BCE particle for a sphere.
    void AddBceSphere(std::shared_ptr<ChBody> body,
                      const ChVector<>& relPos,
                      const ChQuaternion<>& relRot,
                      double radius);

    /// Add BCE particles genetrated from the surface of a sphere.
    void AddBceSphereSurface(std::shared_ptr<ChBody> body,
                             const ChVector<>& relPos,
                             const ChQuaternion<>& relRot,
                             Real radius,
                             Real kernel_h);

    /// Add BCE particle for a cylinder.
    void AddBceCylinder(std::shared_ptr<ChBody> body,
                        const ChVector<>& relPos,
                        const ChQuaternion<>& relRot,
                        double radius,
                        double height,
                        double kernel_h,
                        bool cartesian = true);

    /// Add BCE particles genetrated from the surface of a cylinder.
    void AddBceCylinderSurface(std::shared_ptr<ChBody> body,
                               const ChVector<>& relPos,
                               const ChQuaternion<>& relRot,
                               Real radius,
                               Real height,
                               Real kernel_h);

    /// Add BCE particle for a cone.
    void AddBceCone(std::shared_ptr<ChBody> body,
                    const ChVector<>& relPos,
                    const ChQuaternion<>& relRot,
                    double radius,
                    double height,
                    double kernel_h,
                    bool cartesian = true);

    /// Add BCE particles from a set of points.
    void AddBceFromPoints(std::shared_ptr<ChBody> body,
                          const std::vector<ChVector<>>& points,
                          const ChVector<>& collisionShapeRelativePos,
                          const ChQuaternion<>& collisionShapeRelativeRot);

    /// Add BCE particle from a file.
    void AddBceFile(std::shared_ptr<ChBody> body,
                    const std::string& dataPath,
                    const ChVector<>& collisionShapeRelativePos,
                    const ChQuaternion<>& collisionShapeRelativeRot,
                    double scale,
                    bool isSolid = true);

    /// Add BCE particle from mesh.
    void AddBceFromMesh(std::shared_ptr<fea::ChMesh> my_mesh,
                        const std::vector<std::vector<int>>& NodeNeighborElement,
                        const std::vector<std::vector<int>>& _1D_elementsNodes,
                        const std::vector<std::vector<int>>& _2D_elementsNodes,
                        bool add1DElem,
                        bool add2DElem,
                        bool multiLayer,
                        bool removeMiddleLayer,
                        int SIDE,
                        int SIZE2D);

    /// Add BCE particles genetrated from ANCF shell elements.
    void AddBCE_ShellANCF(std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells,
                          std::shared_ptr<fea::ChMesh> my_mesh,
                          bool multiLayer = true,
                          bool removeMiddleLayer = false,
                          int SIDE = -2);

    /// Add BCE particles genetrated from shell elements.
    void AddBCE_ShellFromMesh(std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& fsiShells,
                              std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& fsiNodes,
                              std::shared_ptr<fea::ChMesh> my_mesh,
                              const std::vector<std::vector<int>>& elementsNodes,
                              const std::vector<std::vector<int>>& NodeNeighborElement,
                              bool multiLayer = true,
                              bool removeMiddleLayer = false,
                              int SIDE = -2);

    /// Create particles from closed mesh
    void CreateMeshMarkers(std::shared_ptr<geometry::ChTriangleMeshConnected> mesh,double delta,
                           std::vector<ChVector<>>& point_cloud);

    /// Create an FSI body for a sphere.
    void CreateSphereFSI(std::shared_ptr<ChMaterialSurface> mat_prop, Real density, const ChVector<>& pos, Real radius);

    /// Create an FSI body for a cylinder.
    void CreateCylinderFSI(std::shared_ptr<ChMaterialSurface> mat_prop,
                           Real density,
                           const ChVector<>& pos,
                           const ChQuaternion<>& rot,
                           Real radius,
                           Real length);

    /// Create an FSI body for a box.
    void CreateBoxFSI(std::shared_ptr<ChMaterialSurface> mat_prop,
                      Real density,
                      const ChVector<>& pos,
                      const ChQuaternion<>& rot,
                      const ChVector<>& hsize);


  private:
    /// Initialize simulation parameters with default values
    void InitParams();

    /// Create BCE particles from the local position on a body.
    void CreateBceGlobalMarkersFromBceLocalPos(const thrust::host_vector<Real4>& posRadBCE,
                                               std::shared_ptr<ChBody> body,
                                               const ChVector<>& collisionShapeRelativePos = ChVector<>(0),
                                               const ChQuaternion<>& collisionShapeRelativeRot = QUNIT,
                                               bool isSolid = true,
                                               bool add_to_fluid_helpers = false,
                                               bool add_to_previous_object = false);

    void CreateBceGlobalMarkersFromBceLocalPos_CableANCF(const thrust::host_vector<Real4>& posRadBCE,
                                                         std::shared_ptr<fea::ChElementCableANCF> cable);

    void CreateBceGlobalMarkersFromBceLocalPos_ShellANCF(const thrust::host_vector<Real4>& posRadBCE,
                                                         std::shared_ptr<fea::ChElementShellANCF_3423> shell,
                                                         double kernel_h = 0);

    /// Create BCE particles from the local position on a boundary.
    void CreateBceGlobalMarkersFromBceLocalPosBoundary(const thrust::host_vector<Real4>& posRadBCE,
                                                       std::shared_ptr<ChBody> body,
                                                       const ChVector<>& collisionShapeRelativePos,
                                                       const ChQuaternion<>& collisionShapeRelativeRot,
                                                       bool isSolid = false,
                                                       bool add_to_previous = true);

    /// Function to initialize the midpoint device data of the fluid system by copying from the full step
    void CopyDeviceDataToHalfStep();

    ChSystemFsi_impl sysFSI;  ///< underlying system implementation
    ChSystem& sysMBS;         ///< reference to the multi-body system

    bool verbose;               ///< enable/disable verbose terminal output (default: true)
    std::string out_dir;        ///< output directory
    OutpuMode file_write_mode;  ///< FSI particle output type (CSV, ChPF, or NONE)

    std::vector<std::shared_ptr<ChBody>> fsiBodies;                        ///< vector of a pointers to FSI bodies
    std::vector<std::shared_ptr<fea::ChElementCableANCF>> fsiCables;       ///< vector of cable ANCF elements
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>> fsiShells;  ///< vector of shell ANCF elements
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> fsiNodes;             ///< vector of FEA nodes
    std::shared_ptr<fea::ChMesh> fsi_mesh;                                 ///< FEA mesh

    std::vector<std::vector<int>> ShellElementsNodes;  ///< indices of nodes of each shell element
    std::vector<std::vector<int>> CableElementsNodes;  ///< indices of nodes of each cable element
    std::shared_ptr<ChFluidDynamics> fluidDynamics;    ///< pointer to the fluid system
    TimeIntegrator fluidIntegrator;                    ///< IISPH by default
    std::shared_ptr<ChFsiInterface> fsiInterface;      ///< pointer to the FSI interface system
    std::shared_ptr<ChBce> bceWorker;                  ///< pointer to the bce workers
    std::shared_ptr<SimParams> paramsH;                ///< pointer to the simulation parameters
    std::shared_ptr<NumberOfObjects> numObjectsH;      ///< number of objects, fluid, bce, and boundary markers

    bool is_initialized;  ///< set to true once the Initialize function is called
    double mTime;         ///< current real time of the simulation

    friend class ChFsiVisualization;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
