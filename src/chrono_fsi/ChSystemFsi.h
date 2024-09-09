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
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceSegmentSet.h"

#include "chrono_fsi/ChApiFsi.h"
#include "chrono_fsi/ChDefinitionsFsi.h"
#include "chrono_fsi/physics/ChFsiInterface.h"
#include "chrono_fsi/math/custom_math.h"

namespace chrono {
namespace fsi {

class ChSystemFsi_impl;
class ChFsiInterface;
class ChFluidDynamics;
class ChBce;
struct SimParams;
struct ChCounters;

/// @addtogroup fsi_physics
/// @{

/// Physical system for fluid-solid interaction problems.
///
/// This class is used to represent fluid-solid interaction problems consisting of fluid dynamics and multibody system.
/// Each of the two underlying physics is an independent object owned and instantiated by this class. The FSI system
/// owns other objects to handle the interface between the two systems, boundary condition enforcing markers, and data.
class CH_FSI_API ChSystemFsi {
  public:
    /// Physics problem type.
    enum class PhysicsProblem { CFD, CRM };

    /// Output mode.
    enum class OutputMode {
        CSV,   ///< comma-separated value
        CHPF,  ///< binary
        NONE   ///< none
    };

    /// Structure with fluid properties.
    /// Used if solving a CFD problem.
    struct CH_FSI_API FluidProperties {
        double density;      ///< fluid density (default: 1000.0)
        double viscosity;    ///< fluid viscosity (default: 0.1)
        double kappa;        ///< surface tension kappa (default: 0)
        double char_length;  ///< characteristic length (default: 1.0)

        FluidProperties();
    };

    /// Structure with elastic material properties.
    /// Used if solving an SPH continuum representation of granular dynamics.
    struct CH_FSI_API ElasticMaterialProperties {
        double density;          ///< bulk density (default: 1000.0)
        double Young_modulus;    ///< Young's modulus (default: 1e6)
        double Poisson_ratio;    ///< Poisson's ratio (default: 0.3)
        double stress;           ///< artifical stress (default: 0)
        double viscosity_alpha;  ///< artifical viscosity coefficient (default: 0.5)
        double viscosity_beta;   ///< artifical viscosity coefficient (default: 0)
        double mu_I0;            ///< reference inertia number (default: 0.03)
        double mu_fric_s;        ///< friction mu_s (default: 0.7)
        double mu_fric_2;        ///< mu_2 constant in mu=mu(I) (default: 0.7)
        double average_diam;     ///< average particle diameter (default: 0.005)
        double friction_angle;   ///< frictional angle of granular material (default: pi/10)
        double dilation_angle;   ///< dilate angle of granular material (default: pi/10)
        double cohesion_coeff;   ///< cohesion coefficient (default: 0)

        ElasticMaterialProperties();
    };

    /// Structure with SPH method parameters.
    struct CH_FSI_API SPHParameters {
        SPHMethod sph_method;             ///< SPH method (default: WCSPH)
        int num_bce_layers;               ///< number of BCE layers (boundary and solids, default: 3)
        double kernel_h;                  ///< kernel separation (default: 0.01)
        double initial_spacing;           ///< initial particle spacing (default: 0.01)
        double max_velocity;              ///< maximum velocity (default: 1.0)
        double xsph_coefficient;          ///< XSPH coefficient (default: 0.5)
        double shifting_coefficient;      ///< shifting beta coefficient (default: 1.0)
        double min_distance_coefficient;  ///< min inter-particle distance as fraction of kernel radius (default: 0.01)
        int density_reinit_steps;         ///< number of steps between density reinitializations (default: 2e8)
        bool use_density_based_projection;         ///< (ISPH only, default: false)
        bool consistent_gradient_discretization;   ///< use G matrix in SPH gradient approximation (default: false)
        bool consistent_laplacian_discretization;  ///< use L matrix in SPH Laplacian approximation (default: false)
        double kernel_threshold;                   ///< threshold for identifying free surface (CRM only, default: 0.8)
        int num_proximity_search_steps;            ///< number of steps between updates to neighbor lists (default: 4)

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

    /// Get ratio of simulation time spent in MBS integration.
    double GetRatioMBS() const { return m_ratio_MBS; }

    /// Enable/disable verbose terminal output.
    void SetVerbose(bool verbose);

    /// Read Chrono::FSI parameters from the specified JSON file.
    void ReadParametersFromFile(const std::string& json_file);

    /// Set initial spacing.
    void SetInitialSpacing(double spacing);

    /// Set SPH kernel length.
    void SetKernelLength(double length);

    /// Set the fluid container dimension
    void SetContainerDim(const ChVector3d& boxDim);

    /// Set periodic boundary condition for fluid.
    void SetBoundaries(const ChVector3d& cMin, const ChVector3d& cMax);

    /// Set half-dimensions of the active domain.
    /// This value activates only those SPH particles that are within an AABB of the specified size from an object
    /// interacting with the "fluid" phase. Note that this setting should *not* be used for actual (CFD) simulations,
    /// but rather oinly when Chrono::FSI is used for continuum representation of granular dynamics (in terramechanics).
    void SetActiveDomain(const ChVector3d& boxHalfDim);

    /// Disable use of the active domain for the given duration at the beginning of the simulation (default: 0).
    /// This parameter is used for settling operations where all particles must be active through the settling process.
    void SetActiveDomainDelay(double duration);

    /// Set number of BCE marker layers (default: 3).
    void SetNumBCELayers(int num_layers);

    /// Set (initial) density.
    void SetDensity(double rho0);

    /// Set prescribed initial pressure for gravity field.
    void SetInitPressure(const double fzDim);

    /// Set gravity for the FSI syatem.
    void SetGravitationalAcceleration(const ChVector3d& gravity);

    /// Set a constant force applied to the fluid.
    /// Solid bodies are not explicitly affected by this force, but they are affected indirectly through the fluid.
    void SetBodyForce(const ChVector3d& force);

    /// Set FSI integration step size.
    void SetStepSize(double dT, double dT_Flex = 0);

    /// Set the maximum allowable integration step size.
    void SetMaxStepSize(double dT_max);

    /// Enable/disable adaptive time stepping.
    void SetAdaptiveTimeStepping(bool adaptive);

    /// Enable/disable SPH integration.
    void SetSPHintegration(bool runSPH);

    /// Set SPH discretization type, consistent or inconsistent
    void SetConsistentDerivativeDiscretization(bool consistent_gradient, bool consistent_Laplacian);

    /// Set cohesion force of the granular material
    void SetCohesionForce(double Fc);

    /// Set the linear system solver for implicit methods.
    //// TODO: OBSOLETE
    void SetSPHLinearSolver(SolverType lin_solver);

    /// Set the SPH method and, optionally, the linear solver type.
    //// TODO: OBSOLETE
    void SetSPHMethod(SPHMethod SPH_method);

    /// Set the number of steps between successive updates to neighbor lists (default: 4).
    void SetNumProximitySearchSteps(int steps);

    /// Enable solution of a CFD problem.
    void SetCfdSPH(const FluidProperties& fluid_props);

    /// Enable solution of elastic SPH (for continuum representation of granular dynamics).
    /// By default, a ChSystemFSI solves an SPH fluid dynamics problem.
    void SetElasticSPH(const ElasticMaterialProperties& mat_props);

    /// Set SPH method parameters.
    void SetSPHParameters(const SPHParameters& sph_params);

    /// Set linear solver parameters (used only for implicit SPH).
    void SetLinSolverParameters(const LinSolverParameters& linsolv_params);

    /// Set simulation data output length
    void SetOutputLength(int OutputLength);

    /// Set the FSI system output mode (default: NONE).
    void SetParticleOutputMode(OutputMode mode) { m_write_mode = mode; }

    /// Return the SPH kernel length of kernel function.
    double GetKernelLength() const;

    /// Return the initial spacing of the SPH particles.
    double GetInitialSpacing() const;

    /// Return the number of BCE layers.
    int GetNumBCELayers() const;

    /// Set the fluid container dimension
    ChVector3d GetContainerDim() const;

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

    /// Return the FSI integration step size.
    double GetStepSize() const;

    /// Return the current value of the maximum allowable integration step size.
    double GetMaxStepSize() const;

    /// Return a flag inicating whether adaptive time stepping is enabled.
    bool GetAdaptiveTimeStepping() const;

    /// Get the number of steps between successive updates to neighbor lists.
    int GetNumProximitySearchSteps() const;

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
    std::vector<ChVector3d> GetParticlePositions() const;

    /// Return the SPH particle velocities.
    std::vector<ChVector3d> GetParticleVelocities() const;

    /// Return the forces acting on SPH particles.
    std::vector<ChVector3d> GetParticleForces() const;

    /// Return the accelerations of SPH particles.
    std::vector<ChVector3d> GetParticleAccelerations() const;

    /// Return the SPH particle fluid properties.
    /// For each SPH particle, the 3-dimensional array contains density, pressure, and viscosity.
    std::vector<ChVector3d> GetParticleFluidProperties() const;

    /// Return the FSI applied force on the body with specified index (as returned by AddFsiBody).
    /// The force is applied at the body COM and is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyForce(size_t i) const;

    /// Return the FSI applied torque on the body with specified index (as returned by AddFsiBody).
    /// The torque is expressed in the absolute frame.
    const ChVector3d& GetFsiBodyTorque(size_t i) const;

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
    void AddSPHParticle(const ChVector3d& point,
                        double rho0,
                        double pres0,
                        double mu0,
                        const ChVector3d& velocity = ChVector3d(0),
                        const ChVector3d& tauXxYyZz = ChVector3d(0),
                        const ChVector3d& tauXyXzYz = ChVector3d(0));

    /// Add an SPH particle with current properties to the SPH system.
    void AddSPHParticle(const ChVector3d& point,
                        const ChVector3d& velocity = ChVector3d(0),
                        const ChVector3d& tauXxYyZz = ChVector3d(0),
                        const ChVector3d& tauXyXzYz = ChVector3d(0));

    /// Create SPH particles in the specified box volume.
    /// The SPH particles are created on a uniform grid with resolution equal to the FSI initial separation.
    void AddBoxSPH(const ChVector3d& boxCenter, const ChVector3d& boxHalfDim);

    // ----------- Functions for adding bodies and associated BCE markers for different shapes

    /// Add a rigid body to the FSI system.
    /// Returns the index of the FSI body in the internal list.
    size_t AddFsiBody(std::shared_ptr<ChBody> body);

    /// Add BCE markers for a rectangular plate of specified X-Y dimensions and associate them with the given body.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// X-Y BCE layers are created in the negative Z direction of the plate orientation frame.
    /// Such a plate is assumed to be used as boundary.
    void AddWallBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector2d& size);

    /// Add BCE markers for a box container of specified dimensions and associate them with the given body.
    /// The center of the box volume is at the origin of the given frame and the the container is aligned with the frame
    /// axes. Such a container is assumed to be used as a boundary.
    /// The 'faces' input vector specifies which faces of the container are to be created: for each direction, a value
    /// of -1 indicates the face in the negative direction, a value of +1 indicates the face in the positive direction,
    /// and a value of 2 indicates both faces. Setting a value of 0 does not create container faces in that direction.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// Such a container is assumed to be used as a fixed boundary and the associated body is not tracked in FSI.
    void AddBoxContainerBCE(std::shared_ptr<ChBody> body,
                            const ChFrame<>& frame,
                            const ChVector3d& size,
                            const ChVector3i faces);

    /// Add BCE markers for a box of specified dimensions and associate them with the given body.
    /// The box is assumed to be centered at the origin of the provided frame and aligned with its axes.
    /// BCE markers are created inside the box if solid=true, and outside the box otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    size_t AddBoxBCE(std::shared_ptr<ChBody> body, const ChFrame<>& frame, const ChVector3d& size, bool solid);

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
    /// The BCE markers are created in the absolute coordinate frame.
    size_t AddPointsBCE(std::shared_ptr<ChBody> body,
                        const std::vector<ChVector3d>& points,
                        const ChFrame<>& rel_frame,
                        bool solid);

    // ----------- Functions for adding FEA meshes and associated BCE markers

    /// Set the BCE marker pattern for 1D flexible solids for subsequent calls to AddFsiMesh.
    /// By default, a full set of BCE markers is used across each section, including a central marker.
    void SetBcePattern1D(BcePatternMesh1D pattern,   ///< marker pattern in cross-section
                         bool remove_center = false  ///< eliminate markers on center line
    );

    /// Set the BCE marker pattern for 2D flexible solids for subsequent calls to AddFsiMesh.
    /// By default, BCE markers are created centered on the mesh surface, with a layer of BCEs on the surface.
    void SetBcePattern2D(BcePatternMesh2D pattern,   ///< pattern of marker locations along normal
                         bool remove_center = false  ///< eliminate markers on surface
    );

    /// Add an FEA mesh to the FSI system.
    /// Contact surfaces (of type segment_set or tri_mesh) already defined for the FEA mesh are used to generate the BCE
    /// markers for the flexible solid. If none are defined, one contact surface of each type is created (as needed) for
    /// the purpose of creating the BCE markers, but these are not attached to the given FEA mesh.
    void AddFsiMesh(std::shared_ptr<fea::ChMesh> mesh);

    // ----------- Utility functions for extracting information at specific SPH particles

    /// Utility function for finding indices of SPH particles inside a given OBB.
    /// The object-oriented box, of specified size, is assumed centered at the origin of the provided frame and aligned
    /// with the axes of that frame. The return value is a device thrust vector.
    thrust::device_vector<int> FindParticlesInBox(const ChFrame<>& frame, const ChVector3d& size);

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
    void CreateBCE_wall(const ChVector2d& size, std::vector<ChVector3d>& bce);

    /// Create BCE markers for a box of specified dimensions, assumed centered at the origin.
    /// BCE markers are created inside the box if solid=true, and outside the box otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    void CreateBCE_box(const ChVector3d& size, bool solid, std::vector<ChVector3d>& bce);

    /// Create BCE markers for a sphere of specified radius, assumed centered at the origin.
    /// BCE markers are created inside the sphere if solid=true, and outside the sphere otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using spherical coordinates (polar=true), or else on a uniform Cartesian grid.
    void CreateBCE_sphere(double rad, bool solid, bool polar, std::vector<ChVector3d>& bce);

    /// Create BCE markers for a cylinder of specified radius and height.
    /// The cylinder is assumed centered at the origin and aligned with the Z axis.
    /// The end-caps are created if capped = true, otherwise the cylinder is open.
    /// BCE markers are created inside the cylinder if solid=true, and outside the cylinder otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (polar=true), or else on a uniform Cartesian grid.
    void CreateBCE_cylinder(double rad,
                            double height,
                            bool solid,
                            bool capped,
                            bool polar,
                            std::vector<ChVector3d>& bce);

    /// Create BCE particles for a cone of specified radius and height.
    /// The cone is assumed centered at the origin and aligned with the Z axis.
    /// The end-cap is created if capped = true, otherwise the cone is open.
    /// BCE markers are created inside the cone if solid=true, and outside the cone otherwise.
    /// BCE markers are created in a number of layers corresponding to system parameters.
    /// BCE markers are created using cylinderical coordinates (polar=true), or else on a uniform Cartesian grid.
    void CreateBCE_cone(double rad, double height, bool solid, bool capped, bool polar, std::vector<ChVector3d>& bce);

    // ----------- Utility functions for creating points filling various volumes

    /// Utility function to create points inside a cylindrical annulus of specified radii and height.
    /// The cylinder annulus is assumed centered at the origin and aligned with the Z axis. If polar = true, points are
    /// created in cylindrical coordinates; otherwise points are created on a uniform Cartesian grid.
    static void CreateCylinderAnnulusPoints(double rad_inner,
                                            double rad_outer,
                                            double height,
                                            bool polar,
                                            double delta,
                                            std::vector<ChVector3d>& points);

    /// Utility function for creating points filling a closed mesh.
    static void CreateMeshPoints(ChTriangleMeshConnected& mesh, double delta, std::vector<ChVector3d>& points);

  public:
    PhysicsProblem GetPhysicsProblem() const;
    std::string GetPhysicsProblemString() const;
    std::string GetSphMethodTypeString() const;

  private:
    /// Initialize simulation parameters with default values.
    void InitParams();

    /// Add a flexible solid with segment set contact to the FSI system.
    void AddFsiMesh1D(std::shared_ptr<fea::ChContactSurfaceSegmentSet> surface);

    /// Add a flexible solid with surface mesh contact to the FSI system.
    void AddFsiMesh2D(std::shared_ptr<fea::ChContactSurfaceMesh> surface);

    /// Create and add BCE markers associated with the given set of contact segments.
    /// The BCE markers are created in the absolute coordinate frame.
    unsigned int AddBCE_mesh1D(unsigned int meshID, const ChFsiInterface::FsiMesh1D& fsi_mesh);

    /// Create and add BCE markers associated with the given mesh contact surface.
    /// The BCE markers are created in the absolute coordinate frame.
    unsigned int AddBCE_mesh2D(unsigned int meshID, const ChFsiInterface::FsiMesh2D& fsi_mesh);

    /// Function to initialize the midpoint device data of the fluid system by copying from the full step.
    void CopyDeviceDataToHalfStep();

    ChSystem* m_sysMBS;  ///< multibody system

    std::shared_ptr<SimParams> m_paramsH;  ///< pointer to the simulation parameters

    bool m_verbose;           ///< enable/disable m_verbose terminal output (default: true)
    std::string m_outdir;     ///< output directory
    OutputMode m_write_mode;  ///< FSI particle output type (CSV, ChPF, or NONE)

    std::unique_ptr<ChSystemFsi_impl> m_sysFSI;         ///< underlying system implementation
    std::unique_ptr<ChFluidDynamics> m_fluid_dynamics;  ///< fluid system
    std::unique_ptr<ChFsiInterface> m_fsi_interface;    ///< FSI interface system
    std::shared_ptr<ChBce> m_bce_manager;               ///< BCE manager

    unsigned int m_num_flex1D_elements;  ///< number of 1-D flexible segments (across all meshes)
    unsigned int m_num_flex2D_elements;  ///< number of 2-D flexible faces (across all meshes)

    unsigned int m_num_flex1D_nodes;  ///< number of 1-D flexible segments (across all meshes)
    unsigned int m_num_flex2D_nodes;  ///< number of 2-D flexible faces (across all meshes)

    std::vector<int> m_fsi_bodies_bce_num;  ///< number of BCE particles on each fsi body

    BcePatternMesh1D m_pattern1D;
    BcePatternMesh2D m_pattern2D;
    bool m_remove_center1D;
    bool m_remove_center2D;

    bool m_is_initialized;  ///< set to true once the Initialize function is called
    bool m_integrate_SPH;   ///< set to true if needs to integrate the fsi solver
    double m_time;          ///< current simulation time

    ChTimer m_timer_step;  ///< timer for integration step
    ChTimer m_timer_MBS;   ///< timer for MBS integration
    double m_RTF;          ///< real-time factor (simulation time / simulated time)
    double m_ratio_MBS;    ///< fraction of step simulation time for MBS integration

    friend class ChFsiVisualizationGL;
    friend class ChFsiVisualizationVSG;
};

/// @} fsi_physics

}  // end namespace fsi
}  // end namespace chrono

#endif
