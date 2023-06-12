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
    ChSystemFsi(ChSystem* sysMBS = nullptr);

    /// Destructor for the FSI system.
    ~ChSystemFsi();

    /// Attach Chrono MBS system.
    void AttachSystem(ChSystem* sysMBS);

    /// Function to integrate the FSI system in time.
    /// It uses a Runge-Kutta 2nd order algorithm to update both the fluid and multibody system dynamics. The midpoint
    /// data of MBS is needed for fluid dynamics update.
    void DoStepDynamics_FSI();

    /// Get current estimated RTF (real time factor).
    double GetRTF() const { return m_RTF; }

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

    /// Set half-dimensions of the active domain.
    /// This value activates only those SPH particles that are within an AABB of the specified size from an object
    /// interacting with the "fluid" phase. Note that this setting should *not* be used for actual (CFD) simulations,
    /// but rather oinly when Chrono::FSI is used for continuum representation of granular dynamics (in terramechanics).
    void SetActiveDomain(const ChVector<>& boxHalfDim);

    /// Disable use of the active domain for the given duration at the beginning of the simulation (default: 0).
    /// This parameter is used for settling operations where all particles must be active through the settling process.
    void SetActiveDomainDelay(double duration);

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

    /// Return the number of BCE layers.
    int GetNumBoundaryLayers() const;

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

    /// Return the current system parameters (debugging only).
    const SimParams& GetParams() const { return *m_paramsH; }

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
    std::vector<std::shared_ptr<ChBody>>& GetFsiBodies() const;

    /// Return the FSI mesh for flexible elements.
    std::shared_ptr<fea::ChMesh> GetFsiMesh() const;

    /// Get a reference to the FSI ChNodeFEAxyzD.
    /// FSI ChNodeFEAxyzD are the ones seen by the fluid dynamics system.
    std::vector<std::shared_ptr<fea::ChNodeFEAxyzD>>& GetFsiNodes() const;

    /// Add a rigid body to the FsiSystem.
    void AddFsiBody(std::shared_ptr<ChBody> body);

    /// Add an FEA mesh to the FSI system.
    void AddFsiMesh(std::shared_ptr<fea::ChMesh> mesh,
                    const std::vector<std::vector<int>>& beam_elements,
                    const std::vector<std::vector<int>>& shell_elements);

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

    // ----------- Functions for adding SPH particles

    /// Add an SPH particle with given properties to the FSI system.
    void AddSPHParticle(const ChVector<>& point,
                        double rho0,
                        double pres0,
                        double mu0,
                        const ChVector<>& velocity = ChVector<>(0),
                        const ChVector<>& tauXxYyZz = ChVector<>(0),
                        const ChVector<>& tauXyXzYz = ChVector<>(0));

    /// Add an SPH particle with current properties to the SPH system.
    void AddSPHParticle(const ChVector<>& point,
                        const ChVector<>& velocity = ChVector<>(0),
                        const ChVector<>& tauXxYyZz = ChVector<>(0),
                        const ChVector<>& tauXyXzYz = ChVector<>(0));

    /// Create SPH particles in the specified box volume.
    /// The SPH particles are created on a uniform grid with resolution equal to the FSI initial separation.
    void AddBoxSPH(const ChVector<>& boxCenter, const ChVector<>& boxHalfDim);

    // ----------- Functions for adding BCE markers in various volumes

    /// Add BCE markers for a rectangular plate of specified X-Y dimensions and associate them with the given body.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// X-Y BCE layers are created in the negative Z direction of the plate orientation frame.
    /// Such a plate is assumed to be used as boundary.
    void AddWallBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector2<> size);

    /// Add BCE markers for a box container of specified dimensions and associate them with the given body.
    /// The center of the box volume is at the origin of the given frame and the the container is aligned with the frame
    /// axes. Such a container is assumed to be used as a boundary.
    /// The 'faces' input vector specifies which faces of the container are to be created: for each direction, a value
    /// of -1 indicates the face in the negative direction, a value of +1 indicates the face in the positive direction,
    /// and a value of 2 indicates both faces. Setting a value of 0 does not create container faces in that direction.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    void AddBoxContainerBCE(std::shared_ptr<ChBody> body,
                            const ChFrame<>& frame,
                            const ChVector<>& size,
                            const ChVector<int> faces);

    /// Add BCE markers for a box of specified dimensions and associate them with the given body.
    /// The box is assumed to be centered at the origin of the provided frame and aligned with its axes.
    /// BCE markers are created inside the box if solid=true, and outside the box otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    size_t AddBoxBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector<>& size, bool solid);

    /// Add BCE markers for a sphere of specified radius and associate them with the given body.
    /// The sphere is assumed to be centered at the origin of the provided frame.
    /// BCE markers are created inside the sphere if solid=true, and outside the sphere otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using spherical coordinates (default), or else on a uniform Cartesian grid.
    size_t AddSphereBCE(std::shared_ptr<ChBody> body,
                        const ChFrame<>& frame,
                        double radius,
                        bool solid,
                        bool polar = true);

    /// Add BCE markers for a cylinder of specified radius and height and associate them with the given body.
    /// The cylinder is assumed centered at the origin of the provided frame and aligned with its Z axis.
    /// BCE markers are created inside the cylinder if solid=true, and outside the cylinder otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (default), or else on a uniform Cartesian grid.
    size_t AddCylinderBCE(std::shared_ptr<ChBody> body,
                          const ChFrame<>& frame,
                          double radius,
                          double height,
                          bool solid,
                          bool capped = true,
                          bool polar = true);

    /// Add BCE markers for a cylindrical annulus of specified radii and height and associate them with the given body.
    /// The cylindrical annulus is assumed centered at the origin of the provided frame and aligned with its Z axis.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (default), or else on a uniform Cartesian grid.
    /// Such a cylindrical annulus is assumed to be used on a solid body.
    size_t AddCylinderAnnulusBCE(std::shared_ptr<ChBody> body,
                                 const ChFrame<>& frame,
                                 double radius_inner,
                                 double radius_outer,
                                 double height,
                                 bool polar = true);

    /// Add BCE markers for a cone of specified radius and height and associate them with the given body.
    /// The cone is assumed centered at the origin of the provided frame and aligned with its Z axis.
    /// BCE markers are created inside the cone if solid=true, and outside the cone otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (default), or else on a uniform Cartesian grid.
    size_t AddConeBCE(std::shared_ptr<ChBody> body,
                      const ChFrame<>& frame,
                      double radius,
                      double height,
                      bool solid,
                      bool capped = true,
                      bool polar = true);

    /// Add BCE markers from a set of points and associate them with the given body.
    /// The points are assumed to be provided relative to the specified frame.
    size_t AddPointsBCE(std::shared_ptr<ChBody> body,
                        const std::vector<ChVector<>>& points,
                        const ChFrame<>& frame,
                        bool solid);

    /// Add BCE markers from mesh.
    //// RADU TODO
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

    // ----------- Utility functions for extracting information at specific SPH particles

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

    // ----------- Utility functions for creating BCE markers in various volumes

    /// Create BCE markers on a rectangular plate of specified X-Y dimensions, assumed centered at the origin.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE layers are created in the negative Z direction.
    void CreateBCE_wall(const Real2& size, thrust::host_vector<Real4>& bce);

    /// Create BCE markers for a box of specified dimensions, assumed centered at the origin.
    /// BCE markers are created inside the box if solid=true, and outside the box otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    void CreateBCE_box(const Real3& size, bool solid, thrust::host_vector<Real4>& bce);

    /// Create BCE markers for a sphere of specified radius, assumed centered at the origin.
    /// BCE markers are created inside the sphere if solid=true, and outside the sphere otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using spherical coordinates (polar=true), or else on a uniform Cartesian grid.
    void CreateBCE_sphere(Real rad, bool solid, bool polar, thrust::host_vector<Real4>& bce);

    /// Create BCE markers for a cylinder of specified radius and height.
    /// The cylinder is assumed centered at the origin and aligned with the Z axis.
    /// The end-caps are created if capped = true, otherwise the cylinder is open.
    /// BCE markers are created inside the cylinder if solid=true, and outside the cylinder otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (polar=true), or else on a uniform Cartesian grid.
    void CreateBCE_cylinder(Real rad,
                            Real height,
                            bool solid,
                            bool capped,
                            bool polar,
                            thrust::host_vector<Real4>& bce);

    /// Create BCE particles for a cylindrical annulus of specified radii and height.
    /// The cylinder annulus is assumed centered at the origin and aligned with the Z axis.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (polar=true), or else on a uniform Cartesian grid.
    void CreateBCE_cylinder_annulus(Real rad_in,
                                    Real rad_out,
                                    Real height,
                                    bool polar,
                                    thrust::host_vector<Real4>& bce);

    /// Create BCE particles for a cone of specified radius and height.
    /// The cone is assumed centered at the origin and aligned with the Z axis.
    /// The end-cap is created if capped = true, otherwise the cone is open.
    /// BCE markers are created inside the cone if solid=true, and outside the cone otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (polar=true), or else on a uniform Cartesian grid.
    void CreateBCE_cone(Real rad, Real height, bool solid, bool capped, bool polar, thrust::host_vector<Real4>& bce);

    /// Create BCE particles from a cable element.
    //// RADU TODO
    void CreateBCE_cable(thrust::host_vector<Real4>& posRadBCE,
                         std::shared_ptr<chrono::fea::ChElementCableANCF> cable,
                         std::vector<int> remove,
                         bool multiLayer,
                         bool removeMiddleLayer,
                         int SIDE);

    /// Create BCE particles from a shell element.
    //// RADU TODO
    void CreateBCE_shell(thrust::host_vector<Real4>& posRadBCE,
                         std::shared_ptr<chrono::fea::ChElementShellANCF_3423> shell,
                         std::vector<int> remove,
                         std::vector<int> remove_s,
                         bool multiLayer,
                         bool removeMiddleLayer,
                         int SIDE);

    /// Utility function for creating points filling a closed mesh.
    //// RADU TODO eliminate delta (use initspacing)
    static void CreateMeshPoints(geometry::ChTriangleMeshConnected& mesh,
                                 double delta,
                                 std::vector<ChVector<>>& point_cloud);

  private:
    /// Initialize simulation parameters with default values.
    void InitParams();

    /// Add the provided BCE markers, expressed in the global frame, to the system.
    /// The BCE markers are assumed to be given in a frame relative to the body reference frame.
    void AddBCE(std::shared_ptr<ChBody> body,
                const thrust::host_vector<Real4>& bce,
                const ChFrame<>& rel_frame,
                bool solid,
                bool add_to_fluid_helpers,
                bool add_to_previous);

    void AddBCE_cable(const thrust::host_vector<Real4>& posRadBCE, std::shared_ptr<fea::ChElementCableANCF> cable);

    void AddBCE_shell(const thrust::host_vector<Real4>& posRadBCE, std::shared_ptr<fea::ChElementShellANCF_3423> shell);

    /// Function to initialize the midpoint device data of the fluid system by copying from the full step.
    void CopyDeviceDataToHalfStep();

    ChSystem* m_sysMBS;  ///< multibody system

    std::shared_ptr<SimParams> m_paramsH;  ///< pointer to the simulation parameters
    TimeIntegrator fluidIntegrator;        ///< IISPH by default

    bool m_verbose;          ///< enable/disable m_verbose terminal output (default: true)
    std::string m_outdir;    ///< output directory
    OutpuMode m_write_mode;  ///< FSI particle output type (CSV, ChPF, or NONE)

    std::unique_ptr<ChSystemFsi_impl> m_sysFSI;         ///< underlying system implementation
    std::unique_ptr<ChFluidDynamics> m_fluid_dynamics;  ///< fluid system
    std::unique_ptr<ChFsiInterface> m_fsi_interface;    ///< FSI interface system
    std::shared_ptr<ChBce> m_bce_manager;               ///< BCE manager

    std::shared_ptr<ChCounters> m_num_objectsH;       ///< number of objects, fluid, bce, and boundary markers
    std::vector<std::vector<int>> m_fea_shell_nodes;  ///< indices of nodes of each shell element
    std::vector<std::vector<int>> m_fea_cable_nodes;  ///< indices of nodes of each cable element

    size_t m_num_cable_elements;
    size_t m_num_shell_elements;

    std::vector<int> m_fsi_bodies_bce_num;  ///< number of BCE particles of each fsi body
    std::vector<int> m_fsi_cables_bce_num;  ///< number of BCE particles of each fsi cable
    std::vector<int> m_fsi_shells_bce_num;  ///< number of BCE particles of each fsi shell

    bool m_is_initialized;  ///< set to true once the Initialize function is called
    bool m_integrate_SPH;   ///< set to true if needs to integrate the fsi solver
    double m_time;          ///< current simulation time

    ChTimer m_timer_step;  ///< timer for integration step
    double m_RTF;          ///< real-time factor (simulation time / simulated time)

    friend class ChFsiVisualizationGL;
    friend class ChFsiVisualizationVSG;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
