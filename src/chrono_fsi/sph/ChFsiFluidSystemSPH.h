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
// Implementation of an FSI-aware SPH fluid solver.
//
// =============================================================================

#ifndef CH_FLUID_SYSTEM_SPH_H
#define CH_FLUID_SYSTEM_SPH_H

#include <map>

#include "chrono_fsi/ChFsiFluidSystem.h"

#include "chrono_fsi/sph/ChFsiParamsSPH.h"
#include "chrono_fsi/sph/ChFsiDefinitionsSPH.h"
#include "chrono_fsi/sph/ChFsiDataTypesSPH.h"

namespace chrono {
namespace fsi {
namespace sph {

class ChFsiInterfaceSPH;
class SphFluidDynamics;
class SphBceManager;
struct FsiDataManager;

/// Device-resident view of the current FSI-SPH marker positions.
/// The positions are stored as Real4 values, with XYZ as world position and W as marker radius.
struct CH_FSI_API ChFsiSphMarkerDeviceView {
    const Real4* pos_rad = nullptr;  ///< device pointer to marker position/radius data
    size_t num_fluid_markers = 0;    ///< number of fluid markers, starting at pos_rad[0]
};

/// @addtogroup fsisph
/// @{

/// Physical system for an FSI-aware SPH fluid solver.
class CH_FSI_API ChFsiFluidSystemSPH : public ChFsiFluidSystem {
  public:
    /// Structure with fluid properties.
    /// Used if solving a CFD problem.
    struct CH_FSI_API FluidProperties {
        double density;      ///< fluid density (default: 1000.0)
        double viscosity;    ///< fluid viscosity (default: 0.1)
        double char_length;  ///< characteristic length (default: 1.0)

        FluidProperties();
    };

    /// Structure with elastic material properties.
    /// Used if solving an SPH continuum representation of granular dynamics.
    struct CH_FSI_API ElasticMaterialProperties {
        double density;              ///< bulk density (default: 1000.0)
        double Young_modulus;        ///< Young's modulus (default: 1e6)
        double Poisson_ratio;        ///< Poisson's ratio (default: 0.3)
        double mu_I0;                ///< reference inertia number (default: 0.03)
        double mu_fric_s;            ///< friction mu_s (default: 0.7)
        double mu_fric_2;            ///< mu_2 constant in mu=mu(I) (default: 0.7)
        double average_diam;         ///< average particle diameter (default: 0.005)
        double cohesion_coeff;       ///< cohesion coefficient (default: 0)
        RheologyCRM rheology_model;  ///< rheology model (default: MU_OF_I)
        double mcc_M;                ///< Cam-Clay critical state line slope, q = M p (default: 0)
        double mcc_kappa;            ///< Cam-Clay swelling index: slope of the elastic
                                     ///< unload/reload line in v-ln(p). Sets the elastic bulk
                                     ///< modulus, K = v p / kappa. Must satisfy
                                     ///< 0 < mcc_kappa < mcc_lambda (default: 0)
        double mcc_lambda;           ///< Cam-Clay compression index: slope of the normal
                                     ///< consolidation line in v-ln(p). Governs virgin
                                     ///< compressibility and the hardening rate, which divides
                                     ///< by (mcc_lambda - mcc_kappa). Must exceed mcc_kappa
                                     ///< (default: 0)
        double mcc_v_lambda;         ///< Specific volume at reference pressure of 1000 Pa
                                     ///< (default: 2.0)

        ElasticMaterialProperties();
    };

    /// Structure with SPH method parameters.
    struct CH_FSI_API SPHParameters {
        IntegrationScheme integration_scheme;          ///< Integration scheme (default: RK2)
        EosType eos_type;                              ///< equation of state (default: ISOTHERMAL)
        ViscosityMethod viscosity_method;              ///< viscosity treatment (default: ARTIFICIAL_UNILATERAL)
        BoundaryMethod boundary_method;                ///< boundary treatment (default: ADAMI)
        KernelType kernel_type;                        ///< kernel type (default: CUBIC_SPLINE)
        ShiftingMethod shifting_method;                ///< shifting method (default: XSPH)
        int num_bce_layers;                            ///< number of BCE layers (boundary and solids, default: 3)
        double initial_spacing;                        ///< initial particle spacing (default: 0.01)
        double d0_multiplier;                          ///< kernel length multiplier, h = d0_multiplier * initial_spacing (default: 1.2)
        double max_velocity;                           ///< maximum velocity (default: 1.0)
        double shifting_xsph_eps;                      ///< XSPH coefficient (default: 0.5)
        double shifting_ppst_push;                     ///< PPST pushing coefficient (default: 3.0)
        double shifting_ppst_pull;                     ///< shifting beta coefficient (default: 1.0)
        double shifting_beta_implicit;                 ///< shifting coefficient used in implicit solver (default: 1.0)
        double shifting_diffusion_A;                   ///< shifting coefficient used in diffusion (default: 1.0, range 1 to 6)
        double shifting_diffusion_AFSM;                ///< shifting coefficient used in diffusion (default: 3.0)
        double shifting_diffusion_AFST;                ///< shifting coefficient used in diffusion (default: 2.0)
        double min_distance_coefficient;               ///< min inter-particle distance as fraction of kernel radius (default: 0.01)
        int density_reinit_steps;                      ///< number of steps between density re-initializations (default: 2e8)
        bool use_density_based_projection;             ///< (ISPH only, default: false)
        bool use_consistent_gradient_discretization;   ///< use G matrix in SPH gradient approximation (default: false)
        bool use_consistent_laplacian_discretization;  ///< use L matrix in SPH Laplacian approximation (default: false)
        double artificial_viscosity;                   ///< artificial viscosity coefficient (default: 0.02)
        bool use_delta_sph;                            ///< use delta SPH (default: true)
        double delta_sph_coefficient;                  ///< delta SPH coefficient (default: 0.1)
        double free_surface_threshold;                 ///< threshold for identifying free surface. The divergence of the position
                                                       ///< field is computed and compared to this threshold. Particles with divergence
                                                       ///< less than this threshold are considered free surface particles (CRM only,
                                                       ///< default: 2.0)
        int num_proximity_search_steps;                ///< number of steps between updates to neighbor lists (default: 1)
        bool use_variable_time_step;                   ///< use variable time step (default: false)

        SPHParameters();
    };

    /// Structure with linear solver parameters (used only for implicit SPH).
    struct CH_FSI_API LinSolverParameters {
        SolverType type;    ///< linear solver type (implicit SPH only, default: JACOBI)
        double atol;        ///< absolute tolerance
        double rtol;        ///< relative tolerance
        int max_num_iters;  ///< maximum number of iterations

        LinSolverParameters();
    };

    /// Structure with surface reconstruction parameters.
    struct CH_FSI_API SplashsurfParameters {
        double smoothing_length;   ///< smoothing length used for the SPH kernel (in multiplies of the particle radius)
        double cube_size;          ///< cube edge length used for marching cubes (in multiplies of the particle radius)
        double surface_threshold;  ///< isosurface threshold for the density  (in multiplies of the rest density)

        SplashsurfParameters();
    };

    ChFsiFluidSystemSPH();
    ~ChFsiFluidSystemSPH();

    /// Enable/disable GPU error checks (default: enabled).
    void EnableGPUErrorCheck(bool val) { m_check_errors = val; }

    /// Set initial spacing.
    void SetInitialSpacing(double spacing);

    /// Set multiplier for interaction length.
    /// h = multiplier * initial_spacing.
    void SetKernelMultiplier(double multiplier);

    /// Set the shifting method.
    void SetShiftingMethod(ShiftingMethod shifting_method);

    /// Set the fluid container dimension.
    void SetContainerDim(const ChVector3d& box_dim);

    /// Set computational domain and boundary conditions on its sides.
    /// `bc_type` indicates the types of BCs imposed in the three directions of the computational domain.
    /// By default, no special boundary conditions are imposed in any direction (BCType::NONE).
    /// \note After Initialize(), only a pure TRANSLATION of the domain is accepted
    /// (same extents; the 2-argument overload additionally requires unchanged BC
    /// types); the update propagates to the device (used by the CRMTerrain moving
    /// patch). A size- or BC-changing post-initialization call throws and leaves the
    /// previous domain intact.
    void SetComputationalDomain(const ChAABB& computational_AABB, BoundaryConditions bc_type);

    /// Set computational domain.
    /// Note that this version leaves the setting for BC type unchanged.
    /// \note After Initialize(), only a pure TRANSLATION of the domain is accepted
    /// (same extents; the 2-argument overload additionally requires unchanged BC
    /// types); the update propagates to the device (used by the CRMTerrain moving
    /// patch). A size- or BC-changing post-initialization call throws and leaves the
    /// previous domain intact.
    void SetComputationalDomain(const ChAABB& computational_AABB);

    /// Set the active domain for the body with specified index (as returned by AddRigidBody).
    /// By default, this is an inverted AABB.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomainBody(size_t i, const ChAABB& aabb);

    /// Set the active domain for nodes of the 1D mesh with specified index (as returned by AddFeaMesh1D).
    /// By default, this is an inverted AABB.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomainMesh1D(size_t i, const ChAABB& aabb);

    /// Set the active domain for nodes of the 2D mesh with specified index (as returned by AddFeaMesh2D).
    /// By default, this is an inverted AABB.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomainMesh2D(size_t i, const ChAABB& aabb);

    /// Set the active domain for all FSI solids (bodies and nodes) to an AABB of given dimensions, centered at the origin.
    /// This active AABB is used for all solids for which an active domain was not set explicitly.
    /// This setting is used only for CRM problems and ignored for CFD problems.
    void SetActiveDomain(const ChVector3d& box_dim);

    /// Specify initial duration of CRM free flow (default: 0).
    /// During this interval, use of FSI solid active domains is disabled unconditionally so that all SPH particles are active
    /// regardless of their position relative to FSI solids. This setting can be used for simulations where the CRM "fluid" is
    /// flowing; e.g., in a dam-break or avalanche simulation, or during an initial settling phase for terramechanics simulations.
    /// If no valid (not inverted) active domain AABB is defined, this setting has no effect (as all particles will be always active).
    void SetFreeFlowDuration(double duration);

    /// Set number of BCE marker layers (default: 3).
    void SetNumBCELayers(int num_layers);

    /// Set (initial) density.
    void SetDensity(double rho0);

    /// Set the PPST Shifting parameters.
    /// - `push`: coefficient for the pushing term in the PPST shifting method (upon penetration with fictitious sphere)
    /// - `pull`: coefficient for the pulling term in the PPST shifting method
    void SetShiftingPPSTParameters(double push, double pull);

    /// Set the XSPH shifting parameters.
    void SetShiftingXSPHParameters(double eps);

    /// Set the diffusion based shifting parameters.
    /// - `A`:    coefficient for the diffusion based shifting method
    /// - `AFSM`: coefficient for the AFSM in the diffusion based shifting method
    /// - `AFST`: coefficient for the AFST in the diffusion based shifting method
    void SetShiftingDiffusionParameters(double A, double AFSM, double AFST);

    /// Set prescribed initial pressure for gravity field.
    void SetInitPressure(const double fzDim);

    /// Set gravity for the FSI system.
    virtual void SetGravitationalAcceleration(const ChVector3d& gravity) override;

    /// Set a constant force applied to the fluid.
    /// Solid bodies are not explicitly affected by this force, but they are affected indirectly through the fluid.
    void SetBodyForce(const ChVector3d& force);

    /// Set SPH discretization type, consistent or inconsistent.
    void SetConsistentDerivativeDiscretization(bool consistent_gradient, bool consistent_Laplacian);

    /// Set cohesion force of the granular material.
    void SetCohesionForce(double Fc);

    /// Set the linear system solver for implicit methods.
    void SetSPHLinearSolver(SolverType lin_solver);

    /// Set the integration scheme (default: RK2).
    /// All explicit integration schemes use a so-called Weakly-Compressible SPH formulation, based on an equation of
    /// state that relates pressure to density.
    void SetIntegrationScheme(IntegrationScheme scheme);

    /// Set the number of steps between successive updates to neighbor lists (default: 4).
    void SetNumProximitySearchSteps(int steps);

    /// Set use variable time step.
    void SetUseVariableTimeStep(bool use_variable_time_step);

    /// Enable solution of a CFD problem.
    void SetCfdSPH(const FluidProperties& fluid_props);

    /// Enable solution of elastic SPH (for continuum representation of granular dynamics).
    /// By default, a ChSystemFSI solves an SPH fluid dynamics problem.
    void SetElasticSPH(const ElasticMaterialProperties& mat_props);

    /// Checks the applicability of user set parameters for SPH and throws an exception if necessary.
    void CheckSPHParameters();

    /// Set SPH method parameters.
    void SetSPHParameters(const SPHParameters& sph_params);

    /// Set linear solver parameters (used only for implicit SPH).
    void SetLinSolverParameters(const LinSolverParameters& linsolv_params);

    /// Set simulation data output level (default: STATE_PRESSURE).
    /// Options:
    /// - STATE           marker state, velocity, and acceleration
    /// - STATE_PRESSURE  STATE plus density and pressure
    /// - CFD_FULL        STATE_PRESSURE plus various CFD parameters
    /// - CRM_FULL        STATE_PRESSURE plus normal and shear stress
    void SetOutputLevel(OutputLevel output_level);

    /// Set boundary treatment type (default: Adami).
    void SetBoundaryType(BoundaryMethod boundary_method);

    /// Set viscosity treatment type (default: artificial unilateral).
    void SetViscosityType(ViscosityMethod viscosity_method);

    /// Set artificial viscosity coefficient (default: 0.02).
    void SetArtificialViscosityCoefficient(double coefficient);

    /// Set kernel type.
    void SetKernelType(KernelType kernel_type);

    /// Return the SPH kernel length of kernel function.
    double GetKernelLength() const;

    /// Return the initial spacing of the SPH particles.
    double GetInitialSpacing() const;

    /// Return the number of BCE layers.
    int GetNumBCELayers() const;

    /// Get the fluid container dimensions.
    ChVector3d GetContainerDim() const;

    /// Get the computational domain.
    ChAABB GetComputationalDomain() const;

    /// Return density.
    double GetDensity() const;

    /// Return viscosity.
    double GetViscosity() const;

    /// Return SPH particle mass.
    double GetParticleMass() const;

    /// Return base pressure.
    double GetBasePressure() const;

    /// Return gravitational acceleration.
    ChVector3d GetGravitationalAcceleration() const;

    /// Return the speed of sound in the fluid phase.
    double GetSoundSpeed() const;

    /// Return the constant force applied to the fluid (if any).
    /// TODO: RENAME
    ChVector3d GetBodyForce() const;

    /// Get the number of steps between successive updates to neighbor lists.
    int GetNumProximitySearchSteps() const;

    /// Get use variable time step
    bool GetUseVariableTimeStep() const;

    /// Return the current system parameters (debugging only).
    const ChFsiParamsSPH& GetParams() const { return *m_paramsH; }

    /// Get the current number of fluid SPH particles.
    size_t GetNumFluidMarkers() const;

    /// Get the current number of boundary BCE markers.
    size_t GetNumBoundaryMarkers() const;

    /// Get the current number of rigid body BCE markers.
    size_t GetNumRigidBodyMarkers() const;

    /// Get the current number of flexible body BCE markers.
    size_t GetNumFlexBodyMarkers() const;

    /// Return the SPH particle positions.
    std::vector<ChVector3d> GetParticlePositions() const;

    /// Return the SPH particle velocities.
    std::vector<ChVector3d> GetParticleVelocities() const;

    /// Return the accelerations of SPH particles.
    std::vector<ChVector3d> GetParticleAccelerations() const;

    /// Return the forces acting on SPH particles.
    std::vector<ChVector3d> GetParticleForces() const;

    /// Return the SPH particle fluid properties.
    /// For each SPH particle, the 3-dimensional vector contains density, pressure, and viscosity.
    std::vector<ChVector3d> GetParticleFluidProperties() const;

    /// Return the boundary treatment type.
    BoundaryMethod GetBoundaryType() const { return m_paramsH->boundary_method; }

    /// Return the viscosity treatment type.
    ViscosityMethod GetViscosityType() const { return m_paramsH->viscosity_method; }

    /// Return the kernel type.
    KernelType GetKernelType() const { return m_paramsH->kernel_type; }

    /// Write FSI system particle output.
    void WriteParticleFile(const std::string& filename) const;

    /// Save current SPH particle and BCE marker data to files.
    /// This function creates three CSV files for SPH particles, boundary BCE markers, and solid BCE markers data.
    void SaveParticleData(const std::string& dir) const;

    /// Save current FSI solid data to files.
    /// This function creates CSV files for force and torque on rigid bodies and flexible nodes.
    void SaveSolidData(const std::string& dir, double time) const;

    // ----------- Functions for adding SPH particles

    /// Interface for callback to set initial particle pressure, density, viscosity, and velocity.
    class CH_FSI_API ParticlePropertiesCallback {
      public:
        ParticlePropertiesCallback() : p0(0), rho0(0), mu0(0), v0(VNULL), tau_diag(VNULL), tau_offdiag(VNULL), consolidation_pressure(0) {}
        ParticlePropertiesCallback(const ParticlePropertiesCallback& other) = default;
        virtual ~ParticlePropertiesCallback() {}

        /// Set values for particle properties.
        /// If an override is provided, it must set *all* particle properties for the current problem type:
        /// - PhysicsProblem::CFD:           p0, v0, rho0, mu0
        /// - PhysicsProblem::CRM (MU_OF_I): p0, v0, rho0, mu0, tau_diag, tau_offdiag
        /// - PhysicsProblem::CRM (MCC):     p0, v0, rho0, mu0, tau_diag, tau_offdiag, consolidation_pressure
        /// The default implementation sets zero velocity and constant density and viscosity.
        /// The default pressure is set to zero, except for CRM with MCC rheology in which case the pressure is set to 1e3.
        virtual void set(const ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) {
            p0 = 0;
            v0 = VNULL;
            rho0 = sysSPH.GetDensity();
            mu0 = sysSPH.GetViscosity();

            if (sysSPH.GetPhysicsProblem() == PhysicsProblem::CRM && sysSPH.GetParams().rheology_model_crm == RheologyCRM::MCC) {
                p0 = 1e3;
                consolidation_pressure = 1.01 * p0;
            }

            tau_diag = ChVector3d(-p0);
            tau_offdiag = VNULL;
        }

        double p0;
        double rho0;
        double mu0;
        ChVector3d v0;
        ChVector3d tau_diag;            ///< CRM only
        ChVector3d tau_offdiag;         ///< CRM only
        double consolidation_pressure;  ///< CRM/MCC only
    };

    /// Add an SPH particle at the specified location with properties provided by the given callback object.
    void AddSPHParticle(const ChVector3d& pos, std::shared_ptr<ParticlePropertiesCallback> props_cb);

    /// Add an SPH particle at the specified location with current FSI-SPH system properties (base pressure, density, and viscosity).
    /// Note that tau_diag and tau_offdiag are only used for CRM problems and consolidation_pressure is only needed for CRM with MCC rheology.
    void AddSPHParticle(const ChVector3d& pos,
                        const ChVector3d& vel = ChVector3d(0),
                        const ChVector3d& tau_diag = ChVector3d(0),
                        const ChVector3d& tau_offdiag = ChVector3d(0),
                        const double consolidation_pressure = 1e3);

    /// Add an SPH particle at the specified location with given properties.
    /// Note that tau_diag and tau_offdiag are only used for CRM problems and consolidation_pressure is only needed for CRM with MCC rheology.
    void AddSPHParticle(const ChVector3d& pos,
                        double density,
                        double pressure,
                        double viscosity,
                        const ChVector3d& vel = ChVector3d(0),
                        const ChVector3d& tau_diag = ChVector3d(0),
                        const ChVector3d& tau_offdiag = ChVector3d(0),
                        const double consolidation_pressure = 1e3);

    /// Create SPH particles in the specified box volume.
    /// The SPH particles are created on a uniform grid with resolution equal to the FSI initial separation.
    /// If not provided, a default ParticlePropertiesCallback callback object is used to set initial particle properties.
    void AddBoxSPH(const ChVector3d& boxCenter, const ChVector3d& boxHalfDim, std::shared_ptr<ParticlePropertiesCallback> params_cb = nullptr);

    // -----------

    /// Add boundary BCE markers at the specified points.
    /// The points are assumed to be provided relative to the specified frame.
    /// These BCE markers are not associated with a particular FSI body and, as such, cannot be used to extract fluid
    /// forces and moments. If fluid reaction forces are needed, create an FSI body with the desired geometry or list
    /// of BCE points and add it through the containing FSI system.
    void AddBCEBoundary(const std::vector<ChVector3d>& points, const ChFramed& frame);

    // ----------- Utility functions for extracting information at specific SPH particles

    /// Utility function for finding indices of SPH particles inside a given OBB.
    /// The object-oriented box, of specified size, is assumed centered at the origin of the provided frame and aligned
    /// with the axes of that frame.
    std::vector<int> FindParticlesInBox(const ChFrame<>& frame, const ChVector3d& size);

    /// Extract positions of all markers (SPH and BCE).
    std::vector<Real3> GetPositions() const;

    /// Return a device-resident view of all marker positions.
    /// This accessor does not copy marker data to the host. The returned device pointer remains owned by this SPH system
    /// and is valid until marker storage is resized or the SPH system is destroyed.
    ChFsiSphMarkerDeviceView GetMarkerDeviceView() const;

    /// Extract velocities of all markers (SPH and BCE).
    std::vector<Real3> GetVelocities() const;

    /// Extract accelerations of all markers (SPH and BCE).
    std::vector<Real3> GetAccelerations() const;

    /// Extract forces applied to all markers (SPH and BCE).
    std::vector<Real3> GetForces() const;

    /// Extract fluid properties of all markers (SPH and BCE).
    /// For each SPH particle, the 3-dimensional vector contains density, pressure, and viscosity.
    std::vector<Real3> GetProperties() const;

    /// Extract positions of all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetPositions(const std::vector<int>& indices) const;

    /// Extract velocities of all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetVelocities(const std::vector<int>& indices) const;

    /// Extract accelerations of all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetAccelerations(const std::vector<int>& indices) const;

    /// Extract forces applied to all markers (SPH and BCE) with indices in the provided array.
    std::vector<Real3> GetForces(const std::vector<int>& indices) const;

    // ----------- Utility functions for creating points in various volumes

    /// Create marker points on a rectangular plate of specified X-Y dimensions, assumed centered at the origin.
    /// Markers are created in a number of layers (in the negative Z direction) corresponding to system parameters.
    std::vector<ChVector3d> CreatePointsPlate(const ChVector2d& size) const;

    /// Create marker points for a box container of specified dimensions.
    /// The box volume is assumed to be centered at the origin. The 'faces' input vector specifies which faces of the
    /// container are to be created: for each direction, a value of -1 indicates the face in the negative direction, a
    /// value of +1 indicates the face in the positive direction, and a value of 2 indicates both faces. Setting a value
    /// of 0 does not create container faces in that direction. Markers are created in a number of layers corresponding
    /// to system parameters.
    std::vector<ChVector3d> CreatePointsBoxContainer(const ChVector3d& size, const ChVector3i& faces) const;

    /// Create interior marker points for a box of specified dimensions, assumed centered at the origin.
    /// Markers are created inside the box, in a number of layers corresponding to system parameters.
    std::vector<ChVector3d> CreatePointsBoxInterior(const ChVector3d& size) const;

    /// Create exterior marker points for a box of specified dimensions, assumed centered at the origin.
    /// Markers are created outside the box, in a number of layers corresponding to system parameters.
    std::vector<ChVector3d> CreatePointsBoxExterior(const ChVector3d& size) const;

    /// Create interior marker points for a sphere of specified radius, assumed centered at the origin.
    /// Markers are created inside the sphere, in a number of layers corresponding to system parameters.
    /// Markers are created using spherical coordinates (polar=true), or else on a uniform Cartesian grid.
    std::vector<ChVector3d> CreatePointsSphereInterior(double radius, bool polar) const;

    /// Create exterior marker points for a sphere of specified radius, assumed centered at the origin.
    /// Markers are created outside the sphere, in a number of layers corresponding to system parameters.
    /// Markers are created using spherical coordinates (polar=true), or else on a uniform Cartesian grid.
    std::vector<ChVector3d> CreatePointsSphereExterior(double radius, bool polar) const;

    /// Create interior marker points for a cylinder of specified radius and height.
    /// The cylinder is assumed centered at the origin and aligned with the Z axis.
    /// Markers are created inside the cylinder, in a number of layers corresponding to system parameters.
    /// Markers are created using cylindrical coordinates (polar=true), or else on a uniform Cartesian grid.
    std::vector<ChVector3d> CreatePointsCylinderInterior(double rad, double height, bool polar) const;

    /// Create exterior marker points for a cylinder of specified radius and height.
    /// The cylinder is assumed centered at the origin and aligned with the Z axis.
    /// Markers are created outside the cylinder, in a number of layers corresponding to system parameters.
    /// Markers are created using cylindrical coordinates (polar=true), or else on a uniform Cartesian grid.
    std::vector<ChVector3d> CreatePointsCylinderExterior(double rad, double height, bool polar) const;

    /// Create interior marker points for a cone of specified radius and height.
    /// The cone is assumed centered at the origin and aligned with the Z axis.
    /// Markers are created inside the cone, in a number of layers corresponding to system parameters.
    /// Markers are created using cylindrical coordinates (polar=true), or else on a uniform Cartesian grid.
    std::vector<ChVector3d> CreatePointsConeInterior(double rad, double height, bool polar) const;

    /// Create interior marker points for a truncated cone of specified radius and height.
    /// The truncated cone is assumed centered at the origin and aligned with the Z axis.
    /// Markers are created inside the truncated cone, in a number of layers corresponding to system parameters.
    /// Markers are created using cylindrical coordinates (polar=true), or else on a uniform Cartesian grid.
    /// The base of the truncated cone has a radius of rad, and the tip has a radius of rad_tip.
    std::vector<ChVector3d> CreatePointsTruncatedConeInterior(double rad, double rad_tip, double height, bool polar) const;

    /// Create exterior marker points for a cone of specified radius and height.
    /// The cone is assumed centered at the origin and aligned with the Z axis.
    /// Markers are created outside the cone, in a number of layers corresponding to system parameters.
    /// Markers are created using cylindrical coordinates (polar=true), or else on a uniform Cartesian grid.
    std::vector<ChVector3d> CreatePointsConeExterior(double rad, double height, bool polar) const;

    /// Create exterior marker points for a truncated cone of specified radius and height.
    /// The truncated cone is assumed centered at the origin and aligned with the Z axis.
    /// Markers are created outside the truncated cone, in a number of layers corresponding to system parameters.
    /// Markers are created using cylindrical coordinates (polar=true), or else on a uniform Cartesian grid.
    /// The base of the truncated cone has a radius of rad, and the tip has a radius of rad_tip.
    std::vector<ChVector3d> CreatePointsTruncatedConeExterior(double rad, double rad_tip, double height, bool polar) const;

    /// Create marker points filling a cylindrical annulus of specified radii and height.
    /// The cylinder annulus is assumed centered at the origin and aligned with the Z axis.
    /// Markers are created using cylindrical coordinates (polar=true), or else on a uniform Cartesian grid.
    std::vector<ChVector3d> CreatePointsCylinderAnnulus(double rad_inner, double rad_outer, double height, bool polar) const;

    /// Create marker points filling a closed mesh.
    /// Markers are created on a Cartesian grid with a separation corresponding to system parameters.
    std::vector<ChVector3d> CreatePointsMesh(ChTriangleMeshConnected& mesh) const;

  public:
    PhysicsProblem GetPhysicsProblem() const;
    std::string GetPhysicsProblemString() const;
    std::string GetSphIntegrationSchemeString() const;

    /// Print the FSI statistics
    void PrintStats() const;

    /// Print the three time step quantities and the final time step to a file
    /// Only valid in variable time step mode
    void PrintTimeSteps(const std::string& path) const;

  public:
    /// Load the given body and mesh node states in the SPH data manager structures.
    /// This function converts FEA mesh states from the provided AOS records to the SOA layout used by the SPH data
    /// manager. LoadSolidStates is always called once during initialization. If the SPH fluid solver is paired with the
    /// generic FSI interface, LoadSolidStates is also called from ChFsiInterfaceGeneric::ExchangeSolidStates at each
    /// co-simulation data exchange. If using the custom SPH FSI interface, MBS states are copied directly to the
    /// device memory in ChFsiInterfaceSPH::ExchangeSolidStates.
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states) override;

    /// Store the body and mesh node forces from the SPH data manager to the given vectors.
    /// If the SPH fluid solver is paired with the generic FSI interface, StoreSolidForces is called from
    /// ChFsiInterfaceGeneric::ExchangeSolidForces at each co-simulation data exchange. If using the custom SPH FSI
    /// interface, MBS forces are copied directly from the device memory in ChFsiInterfaceSPH::ExchangeSolidForces.
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces) override;

#ifdef CHRONO_FEA
    /// Load the given body and mesh node states in the SPH data manager structures.
    /// This function converts FEA mesh states from the provided AOS records to the SOA layout used by the SPH data
    /// manager. LoadSolidStates is always called once during initialization. If the SPH fluid solver is paired with the
    /// generic FSI interface, LoadSolidStates is also called from ChFsiInterfaceGeneric::ExchangeSolidStates at each
    /// co-simulation data exchange. If using the custom SPH FSI interface, MBS states are copied directly to the
    /// device memory in ChFsiInterfaceSPH::ExchangeSolidStates.
    virtual void LoadSolidStates(const std::vector<FsiBodyState>& body_states,
                                 const std::vector<FsiMeshState>& mesh1D_states,
                                 const std::vector<FsiMeshState>& mesh2D_states) override;

    /// Store the body and mesh node forces from the SPH data manager to the given vectors.
    /// If the SPH fluid solver is paired with the generic FSI interface, StoreSolidForces is called from
    /// ChFsiInterfaceGeneric::ExchangeSolidForces at each co-simulation data exchange. If using the custom SPH FSI
    /// interface, MBS forces are copied directly from the device memory in ChFsiInterfaceSPH::ExchangeSolidForces.
    virtual void StoreSolidForces(std::vector<FsiBodyForce>& body_forces, std::vector<FsiMeshForce>& mesh1D_forces, std::vector<FsiMeshForce>& mesh2D_forces) override;
#endif

    // ----------

    /// Function to integrate the fluid system from `time` to `time + step`.
    virtual void OnDoStepDynamics(double time, double step) override;

    /// Get the current step size.
    /// If variable step size is enabled, this returns the current step size (calculated based on system state);
    /// otherwise, it returns the specified constant step size.
    double GetCurrentStepSize() override;

    /// Additional actions taken before applying fluid forces to the solid phase.
    virtual void OnExchangeSolidForces() override;

    /// Additional actions taken after loading new solid phase states.
    virtual void OnExchangeSolidStates() override;

  private:
    /// Derive the domain-dependent grid quantities from the current computational domain.
    void DeriveDomainGridQuantities();

    /// Apply a post-initialization computational-domain update (translation only) and
    /// upload the new parameters to the device.
    void ApplyComputationalDomain(const ChAABB& computational_AABB);

    /// SPH specification of an FSI rigid solid.
    struct FsiSphBody {
        std::shared_ptr<FsiBody> fsi_body;   ///< underlying FSI solid
        std::vector<ChVector3d> bce;         ///< BCE initial global positions
        std::vector<ChVector3d> bce_coords;  ///< local BCE coordinates: (x, y, z) in body frame
        std::vector<int> bce_ids;            ///< BCE identification (body ID)
        bool check_embedded;                 ///< if true, check for overlapping SPH particles
    };

#ifdef CHRONO_FEA
    /// SPH specification of a 1D FSI deformable solid surface.
    struct FsiSphMesh1D {
        std::shared_ptr<FsiMesh1D> fsi_mesh;  ///< underlying FSI solid
        std::vector<ChVector3d> bce;          ///< BCE initial global positions
        std::vector<ChVector3d> bce_coords;   ///< local BCE coordinates: (u, y, z) in segment frame
        std::vector<ChVector3i> bce_ids;      ///< BCE identification (mesh ID, local segment ID, global segment ID)
        bool check_embedded;                  ///< if true, check for overlapping SPH particles
    };

    /// SPH specification of a 2D FSI deformable solid surface.
    struct FsiSphMesh2D {
        std::shared_ptr<FsiMesh2D> fsi_mesh;  ///< underlying FSI solid
        std::vector<ChVector3d> bce;          ///< BCE initial global positions
        std::vector<ChVector3d> bce_coords;   ///< local BCE coordinates: (u, v, z) in triangle frame
        std::vector<ChVector3i> bce_ids;      ///< BCE identification (mesh ID, local face ID, global face ID)
        bool check_embedded;                  ///< if true, check for overlapping SPH particles
    };
#endif

    /// Initialize simulation parameters with default values.
    void InitParams();

    // ----------

    /// SPH solver-specific actions taken when a rigid solid is added as an FSI object.
    virtual void OnAddRigidBody(std::shared_ptr<FsiBody> fsi_body, bool check_embedded) override;

    /// Create the local BCE coordinates, their body associations, and the initial global BCE positions for the given FSI rigid body.
    void CreateRigidBodyBce(std::shared_ptr<FsiBody> fsi_body, std::vector<int>& bce_ids, std::vector<ChVector3d>& bce_coords, std::vector<ChVector3d>& bce);

#ifdef CHRONO_FEA
    /// SPH solver-specific actions taken when a 1D deformable solid is added as an FSI object.
    virtual void OnAddFeaMesh1D(std::shared_ptr<FsiMesh1D> fsi_mesh, bool check_embedded) override;

    /// SPH solver-specific actions taken when a 2D deformable solid is added as an FSI object.
    virtual void OnAddFeaMesh2D(std::shared_ptr<FsiMesh2D> fsi_mesh, bool check_embedded) override;

    /// Set the BCE marker pattern for 1D flexible solids for subsequent calls to AddFeaMesh1D.
    /// By default, a full set of BCE markers is used across each section, including a central marker.
    void SetBcePattern1D(BcePatternMesh1D pattern,  ///< marker pattern in cross-section
                         bool remove_center         ///< eliminate markers on center line
    );

    /// Set the BCE marker pattern for 2D flexible solids for subsequent calls to AddFeaMesh2D.
    /// By default, BCE markers are created centered on the mesh surface, with a layer of BCEs on the surface.
    void SetBcePattern2D(BcePatternMesh2D pattern,  ///< pattern of marker locations along normal
                         bool remove_center         ///< eliminate markers on surface
    );

    /// Create the local BCE coordinates, their mesh associations, and the initial global BCE positions for the given FSI 1D mesh.
    void CreateFeaMesh1DBce(std::shared_ptr<FsiMesh1D> fsi_mesh,
                            BcePatternMesh1D pattern,
                            bool remove_center,
                            std::vector<ChVector3i>& bce_ids,
                            std::vector<ChVector3d>& bce_coords,
                            std::vector<ChVector3d>& bce);

    /// Create the local BCE coordinates, their mesh associations, and the initial global BCE positions for the
    /// given FSI 2D mesh.
    void CreateFeaMesh2DBce(std::shared_ptr<FsiMesh2D> fsi_mesh,
                            BcePatternMesh2D pattern,
                            bool remove_center,
                            std::vector<ChVector3i>& bce_ids,
                            std::vector<ChVector3d>& bce_coords,
                            std::vector<ChVector3d>& bce);
#endif

    // ----------

    /// Initialize the SPH fluid system with FSI support.
    virtual void Initialize(const std::vector<FsiBodyState>& body_states) override;

    /// Add the BCE markers for the given FSI rigid body to the underlying data manager.
    /// Note: BCE markers are created with zero velocities.
    void AddRigidBodyBce(const FsiSphBody& fsisph_body);

#ifdef CHRONO_FEA
    /// Initialize the SPH fluid system with FSI support.
    virtual void Initialize(const std::vector<FsiBodyState>& body_states, const std::vector<FsiMeshState>& mesh1D_states, const std::vector<FsiMeshState>& mesh2D_states) override;

    /// Add the BCE markers for the given FSI 1D mesh to the underlying data manager.
    /// Note: BCE markers are created with zero velocities.
    void AddFeaMesh1DBce(const FsiSphMesh1D& fsisph_mesh);

    /// Add the BCE markers for the given FSI 2D mesh to the underlying data manager.
    /// Note: BCE markers are created with zero velocities.
    void AddFeaMesh2DBce(const FsiSphMesh2D& fsisph_mesh);
#endif

    // ----------

    /// Synchronize the asynchronous copy stream.
    /// Used for the copySortedToOriginal function.
    void SynchronizeCopyStream() const;

    std::shared_ptr<ChFsiParamsSPH> m_paramsH;  ///< simulation parameters
    bool m_force_proximity_search;

    std::unique_ptr<FsiDataManager> m_data_mgr;          ///< FSI data manager
    std::unique_ptr<SphFluidDynamics> m_fluid_dynamics;  ///< fluid system
    std::unique_ptr<SphBceManager> m_bce_mgr;            ///< BCE manager

    unsigned int m_num_rigid_bodies;     ///< number of rigid bodies
    unsigned int m_num_flex1D_meshes;    ///< number of 1-D flexible meshes
    unsigned int m_num_flex2D_meshes;    ///< number of 1-D flexible meshes
    unsigned int m_num_flex1D_nodes;     ///< number of 1-D flexible nodes (across all 1-D meshes)
    unsigned int m_num_flex2D_nodes;     ///< number of 2-D flexible nodes (across all 2-D meshes)
    unsigned int m_num_flex1D_elements;  ///< number of 1-D flexible segments (across all 1-D meshes)
    unsigned int m_num_flex2D_elements;  ///< number of 2-D flexible faces (across all 2-D meshes)

    std::vector<FsiSphBody> m_bodies;  ///< list of FSI rigid bodies
#ifdef CHRONO_FEA
    std::vector<FsiSphMesh1D> m_meshes1D;  ///< list of FSI FEA meshes
    std::vector<FsiSphMesh2D> m_meshes2D;  ///< list of FSI FEA meshes
#endif

    bool m_use_ad;                  ///< use active domains
    bool m_use_default_ad;          ///< a default active domain is defined
    ChAABB m_ad_default;            ///< default active domain
    std::vector<ChAABB> m_ad_body;    ///< body active domains (1 per body)
    std::vector<ChAABB> m_ad_mesh1D;  ///< mesh 1-D node active domains (1 per mesh)
    std::vector<ChAABB> m_ad_mesh2D;  ///< mesh 2-D node active domains (1 per mesh)

    std::vector<int> m_fsi_bodies_bce_num;  ///< number of BCE particles on each fsi body

    OutputLevel m_output_level;
    bool m_check_errors;

    BcePatternMesh1D m_pattern1D;
    BcePatternMesh2D m_pattern2D;
    bool m_remove_center1D;
    bool m_remove_center2D;

    friend class ChFsiSystemSPH;
    friend class ChFsiInterfaceSPH;
    friend class ChFsiProblemSPH;
    friend class ChFsiSplashsurfSPH;
    friend class ChSphVisualizationVSG;
};

// ----------------------------------------------------------------------------

/// Predefined SPH particle initial properties callback (depth-based pressure).
class CH_FSI_API DepthPressurePropertiesCallback : public ChFsiFluidSystemSPH::ParticlePropertiesCallback {
  public:
    DepthPressurePropertiesCallback(double zero_height, ChVector3d init_vel = VNULL) : ParticlePropertiesCallback(), zero_height(zero_height), init_vel(init_vel) {}

    virtual void set(const ChFsiFluidSystemSPH& sysSPH, const ChVector3d& pos) override {
        double gz = std::abs(sysSPH.GetGravitationalAcceleration().z());
        double c2 = sysSPH.GetSoundSpeed() * sysSPH.GetSoundSpeed();

        p0 = sysSPH.GetDensity() * gz * (zero_height - pos.z());
        rho0 = sysSPH.GetDensity() + p0 / c2;
        mu0 = sysSPH.GetViscosity();
        v0 = init_vel;

        // The MCC rheology derives each particle's initial specific volume from log(pc/p), so both the confining pressure
        // and the consolidation pressure must be strictly positive. A depth-based pressure calculation yields exactly zero
        // at the reference height, which is where the topmost particle layer normally sits, so floor the pressure at the
        // overburden of a quarter spacing, using the same spacing that positions the layers.
        // Note that with no gravity along z there is no overburden to derive a confinement from, so reject it here.
        if (sysSPH.GetPhysicsProblem() == PhysicsProblem::CRM && sysSPH.GetParams().rheology_model_crm == RheologyCRM::MCC) {
            ChAssertAlways(gz > 0);
            double p0_min = sysSPH.GetDensity() * gz * sysSPH.GetInitialSpacing() / 4;
            p0 = std::max(p0, p0_min);
            consolidation_pressure = 1.01 * p0;
        }

        tau_diag = ChVector3(-p0);
        tau_offdiag = VNULL;
    }

  private:
    double zero_height;
    ChVector3d init_vel;
};

/// @} fsisph

}  // namespace sph
}  // end namespace fsi
}  // end namespace chrono

#endif
