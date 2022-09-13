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

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>

#include "chrono/ChConfig.h"
#include "chrono/physics/ChSystem.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChDefinitionsFsi.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {

// Forward declarations
namespace fea {
class ChNodeFEAxyzD;
class ChMesh;
class ChElementCableANCF;
class ChElementShellANCF_3423;
}  // namespace fea

namespace fsi {

class ChSystemFsi_impl;
class ChFsiInterface;
class ChFluidDynamics;
class ChBce;
struct SimParams;
struct ChCounters;
struct Real4;

/// @addtogroup fsi_physics
/// @{

/// @brief Physical system for fluid-solid interaction problems.
///
/// This class is used to represent fluid-solid interaction problems consisting of fluid dynamics and multibody system.
/// Each of the two underlying physics is an independent object owned and instantiated by this class. The FSI system
/// owns other objects to handle the interface between the two systems, boundary condition enforcing markers, and data.
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
    struct CH_FSI_API ElasticMaterialProperties {
        double Young_modulus;     ///< Young's modulus
        double Poisson_ratio;     ///< Poisson's ratio
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

    /// Enable/disable m_verbose terminal output.
    void SetVerbose(bool m_verbose);

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

    /// Set size of active domain.
    void SetActiveDomain(const ChVector<>& boxDim);

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

    /// Enable/disable SPH integration.
    void SetSPHintegration(bool runSPH);

    /// Set SPH discretization type, consistent or inconsistent
    void SetDiscreType(bool useGmatrix, bool useLmatrix);

    /// Set wall boundary condition
    void SetWallBC(BceVersion wallBC);

    /// Set rigid body boundary condition
    void SetRigidBodyBC(BceVersion rigidBodyBC);

    /// Set cohesion force of the granular material
    void SetCohesionForce(double Fc);

    /// Set the linear system solver for implicit methods.
    void SetSPHLinearSolver(SolverType lin_solver);

    /// Set the SPH method and, optionally, the linear solver type.
    void SetSPHMethod(FluidDynamics SPH_method, SolverType lin_solver = SolverType::BICGSTAB);

    /// Enable solution of elastic SPH (for continuum representation of granular dynamics).
    /// By default, a ChSystemFSI solves an SPH fluid dynamics problem.
    void SetElasticSPH(const ElasticMaterialProperties mat_props);

    /// Set simulation data output length
    void SetOutputLength(int OutputLength);

    /// Set the FSI system output mode (default: NONE).
    void SetParticleOutputMode(OutpuMode mode) { m_write_mode = mode; }

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
    double GetSimTime() const { return m_time; }

    /// Return the SPH particle positions.
    std::vector<ChVector<>> GetParticlePositions() const;

    /// Return the SPH particle velocities.
    std::vector<ChVector<>> GetParticleVelocities() const;

    /// Return the forces acting on SPH particles.
    std::vector<ChVector<>> GetParticleForces() const;

    /// Return the accelerations of SPH particles.
    std::vector<ChVector<>> GetParticleAccelerations() const;

    /// Return the SPH particle fluid properties.
    /// For each SPH particle, the 3-dimensional array contains density, pressure, and viscosity.
    std::vector<ChVector<>> GetParticleFluidProperties() const;

    /// Get a reference to the FSI bodies.
    /// FSI bodies are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<ChBody>>& GetFsiBodies() { return m_fsi_bodies; }

    /// Return the FSI mesh for flexible elements.
    std::shared_ptr<fea::ChMesh> GetFsiMesh() { return m_fsi_mesh; }

    /// Get a reference to the FSI ChElementCableANCF.
    /// FSI ChElementCableANCF are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementCableANCF>>& GetFsiCables() { return m_fsi_cables; }

    /// Get a reference to the FSI ChElementShellANCF_3423.
    /// FSI ChElementShellANCF_3423 are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>>& GetFsiShells() { return m_fsi_shells; }

    /// Get a reference to the FSI ChNodeFEAxyzD.
    /// FSI ChNodeFEAxyzD are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& GetFsiNodes() { return m_fsi_nodes; }

    /// Add FSI body to the FsiSystem.
    void AddFsiBody(std::shared_ptr<ChBody> mbody) { m_fsi_bodies.push_back(mbody); }

    /// Set number of nodes in FEA cable elements in the FSI system.
    void SetCableElementsNodes(const std::vector<std::vector<int>>& elementsNodes);

    /// Set number of nodes in FEA shell elements in the FSI system.
    void SetShellElementsNodes(const std::vector<std::vector<int>>& elementsNodes);

    /// Set the FSI mesh for flexible elements.
    void SetFsiMesh(std::shared_ptr<fea::ChMesh> other_fsi_mesh);

    /// Complete construction of the FSI system (fluid and BDE objects).
    /// Use parameters read from JSON file and/or specified through various Set functions.
    void Initialize();

    /// Write FSI system particle output.
    void WriteParticleFile(const std::string& outfilename) const;

    /// Save the SPH particle information into files.
    /// This function creates three CSV files for SPH particles, boundary BCE markers, and solid BCE markers data.
    void PrintParticleToFile(const std::string& dir) const;

    /// Save the FSI information into files.
    /// This function creates CSV files for force and torque on rigid bodies and flexible nodes.
    void PrintFsiInfoToFile(const std::string& dir, double time) const;

    /// Add an SPH particle with given properties to the FSI system.
    void AddSPHParticle(const ChVector<>& point,
                        double rho0,
                        double pres0,
                        double mu0,
                        double h,
                        const ChVector<>& velocity = ChVector<>(0),
                        const ChVector<>& tauXxYyZz = ChVector<>(0),
                        const ChVector<>& tauXyXzYz = ChVector<>(0));

    /// Add an SPH particle with current properties to the SPH system.
    void AddSPHParticle(const ChVector<>& point,
                        const ChVector<>& velocity = ChVector<>(0),
                        const ChVector<>& tauXxYyZz = ChVector<>(0),
                        const ChVector<>& tauXyXzYz = ChVector<>(0));

    /// Create SPH particles in the specified box volume.
    /// The SPH particles are created on a uniform grid with given spacing.
    void AddBoxSPH(double initSpace, double kernelLength, const ChVector<>& boxCenter, const ChVector<>& boxHalfDim);

    /// Add BCE markers in a box of given dimensions and at given position associated with the specified body.
    void AddBoxBCE(std::shared_ptr<ChBody> body,
                   const ChVector<>& relPos,
                   const ChQuaternion<>& relRot,
                   const ChVector<>& size,
                   int plane = 12,
                   bool isSolid = false);

    /// Add BCE markers in a sphere of given radius associated with the specified body.
    void AddSphereBCE(std::shared_ptr<ChBody> body,
                      const ChVector<>& relPos,
                      const ChQuaternion<>& relRot,
                      double radius);

    /// Add BCE markers on a spherical surface of given radius associated with the specified body.
    void AddSphereSurfaceBCE(std::shared_ptr<ChBody> body,
                             const ChVector<>& relPos,
                             const ChQuaternion<>& relRot,
                             double radius,
                             double kernel_h);

    /// Add BCE markers in a cylinder of given dimensions and at given position associated with the specified body.
    void AddCylinderBCE(std::shared_ptr<ChBody> body,
                        const ChVector<>& relPos,
                        const ChQuaternion<>& relRot,
                        double radius,
                        double height,
                        double kernel_h,
                        bool cartesian = true);

    /// Add BCE markers on a cylindrical surface of given dimensions and at given position associated with the specified
    /// body.
    void AddCylinderSurfaceBCE(std::shared_ptr<ChBody> body,
                               const ChVector<>& relPos,
                               const ChQuaternion<>& relRot,
                               double radius,
                               double height,
                               double kernel_h);

    /// Add BCE markers in a cone of given dimensions and at given position associated with the specified body.
    void AddConeBCE(std::shared_ptr<ChBody> body,
                    const ChVector<>& relPos,
                    const ChQuaternion<>& relRot,
                    double radius,
                    double height,
                    double kernel_h,
                    bool cartesian = true);

    /// Add BCE markers from a set of points and associate them with the given body.
    void AddPointsBCE(std::shared_ptr<ChBody> body,
                      const std::vector<ChVector<>>& points,
                      const ChVector<>& collisionShapeRelativePos,
                      const ChQuaternion<>& collisionShapeRelativeRot);

    /// Add BCE markers read from the specified file andd associate them with the given body.
    void AddFileBCE(std::shared_ptr<ChBody> body,
                    const std::string& dataPath,
                    const ChVector<>& collisionShapeRelativePos,
                    const ChQuaternion<>& collisionShapeRelativeRot,
                    double scale,
                    bool isSolid = true);

    /// Add BCE markers from mesh.
    void AddFEAmeshBCE(std::shared_ptr<fea::ChMesh> my_mesh,
                       const std::vector<std::vector<int>>& NodeNeighborElement,
                       const std::vector<std::vector<int>>& _1D_elementsNodes,
                       const std::vector<std::vector<int>>& _2D_elementsNodes,
                       bool add1DElem,
                       bool add2DElem,
                       bool multiLayer,
                       bool removeMiddleLayer,
                       int SIDE,
                       int SIZE2D);

    /// Create and add to the FSI system a rigid body with spherical shape.
    /// BCE markers are created in the entire spherical volume using the current spacing value.
    void AddSphereBody(std::shared_ptr<ChMaterialSurface> mat_prop,
                       double density,
                       const ChVector<>& pos,
                       double radius);

    /// Create and add to the FSI system a rigid body with cylindrical shape.
    /// BCE markers are created in the entire cylindrical volume using the current spacing value.
    void AddCylinderBody(std::shared_ptr<ChMaterialSurface> mat_prop,
                         double density,
                         const ChVector<>& pos,
                         const ChQuaternion<>& rot,
                         double radius,
                         double length);

    /// Create and add to the FSI system a rigid body with box shape.
    /// BCE markers are created in the entire box volume using the current spacing value.
    void AddBoxBody(std::shared_ptr<ChMaterialSurface> mat_prop,
                    double density,
                    const ChVector<>& pos,
                    const ChQuaternion<>& rot,
                    const ChVector<>& hsize);

    /// Utility function for creating points filling a closed mesh.
    static void CreateMeshPoints(geometry::ChTriangleMeshConnected& mesh,
                                 double delta,
                                 std::vector<ChVector<>>& point_cloud);

    /// Utility function for finding indices of SPH particles inside a given OBB.
    /// The object-oriented box, of specified size, is assumed centered at the origin of the provided frame and aligned
    /// with the axes of that frame. The return value is a device thrust vector.
    thrust::device_vector<int> FindParticlesInBox(const ChFrame<>& frame, const ChVector<>& size);

    /// Extract positions of all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real4> GetParticlePositions(const thrust::device_vector<int>& indices);

    /// Extract velocities of all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real3> GetParticleVelocities(const thrust::device_vector<int>& indices);

    /// Extract forces applied to all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real4> GetParticleForces(const thrust::device_vector<int>& indices);

    /// Extract accelerations of all SPH particles with indices in the provided array.
    /// The return value is a device thrust vector.
    thrust::device_vector<Real4> GetParticleAccelerations(const thrust::device_vector<int>& indices);

  private:
    /// Initialize simulation parameters with default values.
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

    /// Function to initialize the midpoint device data of the fluid system by copying from the full step.
    void CopyDeviceDataToHalfStep();

    ChSystem& m_sysMBS;  ///< reference to the multi-body system

    std::shared_ptr<SimParams> m_paramsH;  ///< pointer to the simulation parameters
    TimeIntegrator fluidIntegrator;        ///< IISPH by default

    bool m_verbose;          ///< enable/disable m_verbose terminal output (default: true)
    std::string m_outdir;    ///< output directory
    OutpuMode m_write_mode;  ///< FSI particle output type (CSV, ChPF, or NONE)

    std::vector<std::shared_ptr<ChBody>> m_fsi_bodies;                        ///< vector of a pointers to FSI bodies
    std::vector<std::shared_ptr<fea::ChElementCableANCF>> m_fsi_cables;       ///< vector of cable ANCF elements
    std::vector<std::shared_ptr<fea::ChElementShellANCF_3423>> m_fsi_shells;  ///< vector of shell ANCF elements
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>> m_fsi_nodes;             ///< vector of FEA nodes
    std::shared_ptr<fea::ChMesh> m_fsi_mesh;                                  ///< FEA mesh

    std::unique_ptr<ChSystemFsi_impl> m_sysFSI;         ///< underlying system implementation
    std::unique_ptr<ChFluidDynamics> m_fluid_dynamics;  ///< fluid system
    std::unique_ptr<ChFsiInterface> m_fsi_interface;    ///< FSI interface system
    std::shared_ptr<ChBce> m_bce_manager;               ///< BCE manager

    std::shared_ptr<ChCounters> m_num_objectsH;       ///< number of objects, fluid, bce, and boundary markers
    std::vector<std::vector<int>> m_fea_shell_nodes;  ///< indices of nodes of each shell element
    std::vector<std::vector<int>> m_fea_cable_nodes;  ///< indices of nodes of each cable element

    std::vector<int> m_fsi_bodies_bce_num;  ///< number of BCE particles of each fsi body
    std::vector<int> m_fsi_cables_bce_num;  ///< number of BCE particles of each fsi cable
    std::vector<int> m_fsi_shells_bce_num;  ///< number of BCE particles of each fsi shell

    bool m_is_initialized;  ///< set to true once the Initialize function is called
    bool m_integrate_SPH;   ///< set to true if needs to integrate the fsi solver
    double m_time;          ///< current real time of the simulation

    friend class ChVisualizationFsi;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
