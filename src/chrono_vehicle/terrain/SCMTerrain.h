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
// Authors: Alessandro Tasora, Radu Serban, Jay Taves
// =============================================================================
//
// Deformable terrain based on SCM (Soil Contact Model) from DLR
// (Krenn & Hirzinger)
//
// =============================================================================

#ifndef SCM_TERRAIN_H
#define SCM_TERRAIN_H

#include <cstdint>
#include <string>
#include <ostream>
#include <bitset>
#include <cassert>
#include <memory>
#include <unordered_map>

#include "chrono/core/ChTimer.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/assets/ChColormap.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadsNodeXYZ.h"
#include "chrono/physics/ChSystem.h"
#ifdef CHRONO_FEA
    #include "chrono/fea/ChNodeFEAxyz.h"
#endif

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"

#ifdef CHRONO_HAS_SCM_GPU
    #include "chrono_vehicle/terrain/SCMGpu.h"
    #include "chrono_vehicle/terrain/SCMRaycastGpu.h"
#endif

namespace chrono {
namespace vehicle {

class SCMLoader;

#ifdef CHRONO_HAS_SCM_GPU
namespace scm_gpu {
struct ScmHitRecord {
    ChContactable* contactable = nullptr;
    ChVector3d abs_point;
    int patch_id = -1;
};
void PrimeBuffers();
}  // namespace scm_gpu
#endif

/// Raw ray-cast hit produced by the GPU ray-cast reference backend.
/// Deliberately independent of CHRONO_HAS_SCM_GPU: this is plain data, no HIP dependency, and is
/// consumed locally by SCMLoader::ComputeInternalForces() regardless of the contact-force GPU flag.
struct RaycastHit {
    ChVector2i ij;
    ChContactable* contactable;
    ChVector3d abs_point;
};

/// @addtogroup vehicle_terrain
/// @{

/// Deformable terrain model.
/// This class implements a deformable terrain based on the Soil Contact Model.
/// Unlike RigidTerrain, the vertical coordinates of this terrain mesh can be deformed
/// due to interaction with ground vehicles or other collision shapes.
class CH_VEHICLE_API SCMTerrain : public ChTerrain {
  public:
    enum DataPlotType {
        PLOT_NONE,
        PLOT_LEVEL,
        PLOT_LEVEL_INITIAL,
        PLOT_SINKAGE,
        PLOT_SINKAGE_ELASTIC,
        PLOT_SINKAGE_PLASTIC,
        PLOT_STEP_PLASTIC_FLOW,
        PLOT_PRESSURE,
        PLOT_PRESSURE_YIELD,
        PLOT_SHEAR,
        PLOT_K_JANOSI,
        PLOT_IS_TOUCHED,
        PLOT_ISLAND_ID,
        PLOT_MASSREMAINDER
    };

    /// Information at SCM node.
    struct NodeInfo {
        double sinkage;          ///< sinkage, along local normal direction
        double sinkage_plastic;  ///< sinkage due to plastic deformation, along local normal direction
        double sinkage_elastic;  ///< sinkage due to plastic deformation, along local normal direction
        double sigma;            ///< normal pressure, along local normal direction
        double sigma_yield;      ///< yield pressure, along local normal direction
        double kshear;           ///< Janosi-Hanamoto shear, along local tangent direction
        double tau;              ///< shear stress, along local tangent direction

        NodeInfo() = default;
    };

    /// Construct a default SCM deformable terrain.
    /// The user is responsible for calling various Set methods before Initialize.
    SCMTerrain(ChSystem* system,               ///< [in] containing multibody system
               bool visualization_mesh = true  ///< [in] enable/disable visualization asset
    );

    ~SCMTerrain() {}

    /// Set the SCM reference frame.
    /// By default, the reference frame is aligned with the global ISO vehicle reference frame.
    /// To set as Y up, call SetReferenceFrame(ChCoordys(VNULL, QuatFromAngleX(-CH_PI_2)));
    void SetReferenceFrame(const ChCoordsys<>& plane);

    /// Get the current SCM reference frame.
    /// The SCM terrain patch is defined relative to the (x,y) plane of this frame, with normal along the Z axis.
    const ChCoordsys<>& GetReferenceFrame() const;

    /// Set the properties of the SCM soil model.
    /// These parameters are described in: "Parameter Identification of a Planetary Rover Wheel-Soil Contact Model via a
    /// Bayesian Approach", A.Gallina, R. Krenn et al. Note that the original SCM model does not include the K and R
    /// coefficients. A very large value of K and R=0 reproduce the original SCM.
    void SetSoilParameters(double Bekker_Kphi,    ///< Kphi, frictional modulus in Bekker model
                           double Bekker_Kc,      ///< Kc, cohesive modulus in Bekker model
                           double Bekker_n,       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
                           double Mohr_cohesion,  ///< Cohesion for shear failure [Pa]
                           double Mohr_friction,  ///< Friction angle for shear failure [degree]
                           double Janosi_shear,   ///< Shear parameter in Janosi-Hanamoto formula [m]
                           double elastic_K,      ///< elastic stiffness K per unit area, [Pa/m] (must be larger than Kphi)
                           double damping_R       ///< vertical damping R per unit area [Pa.s/m] (proportional to vertical speed)
    );

    /// Enable/disable the creation of soil inflation at the side of the ruts (bulldozing effects).
    void EnableBulldozing(bool mb);

    /// Enable/disable the CPU reference implementation of the GPU ray-cast backend.
    /// Replaces the ray-cast loop with an equivalent mesh-rasterization pass
    /// over ChBody-derived contactables that have a triangle-mesh collision shape, producing the same
    /// {contactable, abs_point} hit data as the default loop. Requires explicit per-body active domains
    /// (AddActiveDomain). Intended for validating the HIP backend against an equivalent CPU
    /// implementation; the default ray-cast loop remains the production CPU path.
    void EnableRaycastGpuReference(bool val);

    /// Set parameters controlling the creation of side ruts (bulldozing effects).
    void SetBulldozingParameters(double erosion_angle,          ///< angle of erosion of the displaced material [degrees]
                                 double flow_factor = 1.0,      ///< growth of lateral volume relative to pressed volume
                                 int erosion_iterations = 3,    ///< number of erosion refinements per timestep
                                 int erosion_propagations = 10  ///< number of concentric vertex selections subject to erosion
    );

    /// Set the vertical level up to which collision is tested (relative to the reference level at the sample point).
    /// Since the contact is unilateral, this could be zero. However, when computing bulldozing flow, one might also
    /// need to know if in the surrounding there is some potential future contact: so it might be better to use a
    /// positive value (but not higher than the max. expected height of the bulldozed rubble, to avoid slowdown of
    /// collision tests). Default: 0.1 m.
    void SetTestHeight(double offset);

    ///  Return the current test height level.
    double GetTestHeight() const;

    /// Set the color plot type for the SCM mesh.
    /// Specify the minimum and maximum values for false coloring.
    void SetPlotType(DataPlotType plot_type, double min_val, double max_val);

    /// Set the colormap type for false coloring of the SCM mesh.
    /// The default colormap is JET (a divergent blue-red map).
    void SetColormap(ChColormap::Type type);

    /// Get the type of the colormap currently in use.
    ChColormap::Type GetColormapType() const;

    /// Get the colormap object in current use.
    const ChColormap& GetColormap() const;

    /// Set visualization color.
    void SetColor(const ChColor& color);

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float scale_x = 1,           ///< [in] texture X scale
                    float scale_y = 1            ///< [in] texture Y scale
    );

    /// Set boundary of the SCM computational domain.
    /// By default, the SCM terrain patch extends to infinity in the x-y plane, beyond the area used to initialize it;
    /// outside the initialization area, the height of the SCM terrain is that of the closest initialized point.
    /// By specifying a boundary, SCM terrain forces outside that boundary are not generated. This feature is useful in
    /// stitching an environment with multiple SCM terrain patches or with a combination of SCM and rigid terrain
    /// patches. The boundary is specified as an axis-aligned bounding box expressed relative to the SCM reference
    /// plane. Note that the z values of the provided AABB are not used (as long as the AABB is not inverted).
    void SetBoundary(const ChAABB& aabb);

    /// Add a new moving active domain associated with the specified body.
    /// Note: the OOBB is placed relative to the body *reference frame*.
    /// Multiple calls to this function can be made, each of them adding a new active domain.
    /// The union of all currently defined active domains is used to reduce the number of ray casting operations, by
    /// ensuring that rays are generated only from SCM grid nodes inside the projection of the an active domains's OOBB
    /// onto the SCM reference plane. If there are no user-provided active domains, a single default one is defined to
    /// encompass all collision shapes in the system at any given time.
    void AddActiveDomain(std::shared_ptr<ChBody> body,   ///< [in] monitored body
                         const ChVector3d& OOBB_center,  ///< [in] OOBB center, relative to body reference frame
                         const ChVector3d& OOBB_dims     ///< [in] OOBB dimensions
    );

    /// Remove every active domain registered against the specified body.
    ///
    /// Intended for bodies that have gone permanently static: a domain's whole purpose is to make
    /// SCM compute soil under a body that can move through it, and a body that can no longer move
    /// needs no soil reaction. Keeping its domain is not free -- every node the domain covers is
    /// re-probed against the (monotonically growing) node map on every step for the rest of the
    /// run -- so a simulation that accumulates static bodies accumulates per-step cost with them.
    ///
    /// Two cautions. Soil is computed ONLY inside active domains, so a body whose domain is removed
    /// and which then becomes dynamic again will receive no soil support and will fall through the
    /// terrain: removal must be paired with re-adding on the reverse transition. And removing the
    /// last user domain leaves the terrain with none at all (it does not revert to the default
    /// whole-scene domain that AddActiveDomain discarded), which means no ray casts and no
    /// deformation anywhere.
    ///
    /// Returns the number of domains removed; 0 if the body had none.
    std::size_t RemoveActiveDomain(std::shared_ptr<ChBody> body);

    /// Return the number of currently registered active domains.
    std::size_t GetNumActiveDomains() const;

    /// Class to be used as a callback interface for location-dependent soil parameters.
    /// A derived class must implement Set() and set *all* soil parameters (no defaults are provided).
    class CH_VEHICLE_API SoilParametersCallback {
      public:
        virtual ~SoilParametersCallback() {}

        /// Set the soil properties at a given (x,y) location (below the given point).
        /// Attention: the location is assumed to be provided in the SCM reference frame!
        virtual void Set(const ChVector3d& loc,  ///< query location
                         double& Bekker_Kphi,    ///< frictional modulus in Bekker model
                         double& Bekker_Kc,      ///< cohesive modulus in Bekker model
                         double& Bekker_n,       ///< exponent of sinkage in Bekker model (usually 0.6...1.8)
                         double& Mohr_cohesion,  ///< cohesion for shear failure [Pa]
                         double& Mohr_friction,  ///< friction angle for shear failure [degree]
                         double& Janosi_shear,   ///< shear parameter in Janosi-Hanamoto formula [m]
                         double& elastic_K,      ///< elastic stiffness K per unit area, [Pa/m] (must be larger than Kphi)
                         double& damping_R       ///< vertical damping R per unit area [Pa.s/m] (proportional to vertical speed)
                         ) = 0;
    };

    /// Specify the callback object to set the soil parameters at given (x,y) locations.
    /// To use constant soil parameters throughout the entire patch, use SetSoilParameters.
    void RegisterSoilParametersCallback(std::shared_ptr<SoilParametersCallback> cb);

    /// Get the initial (undeformed) terrain height below the specified location.
    double GetInitHeight(const ChVector3d& loc) const;

    /// Get the initial (undeformed) terrain normal at the point below the specified location.
    ChVector3d GetInitNormal(const ChVector3d& loc) const;

    /// Get the point on the terrain below the specified location.
    virtual ChVector3d GetPoint(const ChVector3d& loc) const override;

    /// Get the terrain height below the specified location.
    virtual double GetHeight(const ChVector3d& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual ChVector3d GetNormal(const ChVector3d& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For SCMTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value of 0.8.
    virtual float GetCoefficientFriction(const ChVector3d& loc) const override;

    /// Get SCM information at the node closest to the specified location.
    NodeInfo GetNodeInfo(const ChVector3d& loc) const;

    /// Get the visualization triangular mesh.
    std::shared_ptr<ChVisualShapeTriangleMesh> GetMesh() const;

    /// Set the visualization mesh as wireframe or as solid (default: wireframe).
    /// Note: in wireframe mode, normals for the visualization mesh are not calculated.
    void SetMeshWireframe(bool val);

    /// Save the visualization mesh as a Wavefront OBJ file.
    void WriteMesh(const std::string& filename) const;

    /// Enable/disable co-simulation mode (default: false).
    /// In co-simulation mode, the underlying SCM loader does not apply loads to interacting objects.
    /// Instead, contact forces are accumulated and available for extraction using GetContactForceBody and
    /// GetContactForceNode for rigid bodies and FEA nodes, respectively.
    void SetCosimulationMode(bool val);

#ifdef CHRONO_HAS_SCM_GPU
    /// Enable/disable the HIP contact-force backend (default: enabled when built with SCM GPU support).
    void SetScmGpuEnabled(bool enable);

    /// Return whether the HIP contact-force backend is enabled.
    bool IsScmGpuEnabled() const;

    /// Set runtime tuning parameters for the SCM GPU backend.
    void SetScmGpuConfig(const scm_gpu::Config& config);

    /// Get the current SCM GPU backend configuration.
    scm_gpu::Config GetScmGpuConfig() const;

    /// Enable/disable the HIP ray-cast backend.
    ///
    /// Default: enabled, in builds configured with the SCM GPU backend. The backend additionally
    /// requires explicit per-body active domains (AddActiveDomain); without them ray-casting silently
    /// uses the CPU path, so leaving this on is always safe. Call with false to force the CPU path.
    void EnableRaycastGpuHip(bool val);
#endif

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double sizeX,  ///< [in] terrain dimension in the X direction
                    double sizeY,  ///< [in] terrain dimension in the Y direction
                    double delta   ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed terrain profile is provided via the specified image file as a height map.
    /// The terrain patch is scaled in the horizontal plane of the SCM frame to sizeX x sizeY, while the initial height
    /// is scaled between hMin and hMax (with the former corresponding to a pure black pixel and the latter to a pure
    /// white pixel).  The SCM grid resolution is specified through 'delta' and initial heights at grid points are
    /// obtained through interpolation (outside the terrain patch, the SCM node height is initialized to the height of
    /// the closest image pixel). For visualization purposes, a triangular mesh is also generated from the provided
    /// image file.
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (image file)
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax,                        ///< [in] maximum height (white level)
                    double delta                        ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed terrain profile is provided via the specified Wavefront OBJ mesh file.
    /// The dimensions of the terrain patch in the horizontal plane of the SCM frame is set to the range of the x and y
    /// mesh vertex coordinates, respectively.  The SCM grid resolution is specified through 'delta' and initial heights
    /// at grid points are obtained through linear interpolation (outside the mesh footprint, the height of a grid node
    /// is set to the height of the closest point on the mesh).  A visualization mesh is created from the original mesh
    /// resampled at the grid node points.
    void Initialize(const std::string& mesh_file,  ///< [in] filename for the mesh (Wavefront OBJ)
                    double delta                   ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed terrain profile is provided via the specified triangular mesh.
    /// The dimensions of the terrain patch in the horizontal plane of the SCM frame is set to the range of the x and y
    /// mesh vertex coordinates, respectively.  The SCM grid resolution is specified through 'delta' and initial heights
    /// at grid points are obtained through linear interpolation (outside the mesh footprint, the height of a grid node
    /// is set to the height of the closest point on the mesh).  A visualization mesh is created from the original mesh
    /// resampled at the grid node points.
    void Initialize(const ChTriangleMeshConnected& trimesh,  ///< [in] surface triangular mesh
                    double delta                             ///< [in] grid spacing
    );

    /// Node height level at a given grid location.
    typedef std::pair<ChVector2i, double> NodeLevel;

    /// Get the heights of all modified grid nodes.
    /// If 'all_nodes = true', return modified nodes from the start of simulation.  Otherwise, return only the nodes
    /// modified over the last step.
    std::vector<NodeLevel> GetModifiedNodes(bool all_nodes = false) const;

    /// Modify the level of grid nodes from the given list.
    void SetModifiedNodes(const std::vector<NodeLevel>& nodes);

    /// Number of steps where the GPU ray-cast backend actually produced the hits.
    /// The backend falls back to the CPU per step when a model or a step does not qualify, and that fallback is deliberately silent.
    /// Calls to this function check if the GPU backend was used.
    int GetNumRaycastGpuSteps() const;

    /// Number of steps where the GPU contact-force backend actually computed the forces.
    /// The backend falls back to the CPU per step when a model or a step does not qualify, and that fallback is deliberately silent.
    /// Calls to this function check if the GPU backend was used.
    int GetNumContactForceGpuSteps() const;

    /// Return the cumulative contact force on the specified body  (due to interaction with the SCM terrain).
    /// The return value is true if the specified body experiences contact forces and false otherwise.
    /// If contact forces are applied to the body, they are reduced to the body center of mass.
    bool GetContactForceBody(std::shared_ptr<ChBody> body, ChVector3d& force, ChVector3d& torque) const;

#ifdef CHRONO_FEA
    /// Return the cumulative contact force on the specified mesh node (due to interaction with the SCM terrain).
    /// The return value is true if the specified node experiences contact forces and false otherwise.
    bool GetContactForceNode(std::shared_ptr<fea::ChNodeFEAxyz> node, ChVector3d& force) const;
#endif

    /// Return the number of rays cast at last step.
    int GetNumRayCasts() const;
    /// Return the number of ray hits at last step.
    int GetNumRayHits() const;
    /// Return the number of contact patches at last step.
    int GetNumContactPatches() const;
    /// Return the number of nodes in the erosion domain at last step (bulldozing effects).
    int GetNumErosionNodes() const;

    /// Return the number of grid nodes deformed since the run began.
    /// This is the size of the sparse node map, which only ever grows: a node enters it the first
    /// time a ray hits it and is never evicted. It is worth watching because the per-step cost of
    /// ray casting is one hash probe of this map per node of every active domain, so once the map
    /// outgrows the last-level cache each probe becomes a DRAM access and the whole simulation
    /// slows down in proportion to ground covered rather than to anything physical.
    std::size_t GetNumDeformedNodes() const;

    /// Return time for updating active domains at last step (ms).
    double GetTimerActiveDomains() const;
    /// Return time for geometric ray intersection tests at last step (ms).
    double GetTimerRayTesting() const;
    /// Return time for ray casting at last step (ms). Includes time for ray intersection testing.
    double GetTimerRayCasting() const;
    /// Return time for computing contact patches at last step (ms).
    double GetTimerContactPatches() const;
    /// Return time for computing contact forces at last step (ms).
    double GetTimerContactForces() const;
    /// Return time for computing bulldozing effects at last step (ms).
    double GetTimerBulldozing() const;
    /// Return time for visualization assets update at last step (ms).
    double GetTimerVisUpdate() const;

    /// Print timing and counter information for last step.
    void PrintStepStatistics(std::ostream& os) const;

    std::shared_ptr<SCMLoader> GetSCMLoader() const { return m_loader; }

    void SetBaseMeshLevel(double level);

  private:
    std::shared_ptr<SCMLoader> m_loader;  ///< underlying load container for contact force generation

    friend class ChScmVisualizationVSG;
};

/// Parameters for soil-contactable interaction.
class CH_VEHICLE_API SCMContactableData {
  public:
    SCMContactableData(double area_ratio,     ///< area fraction with overridden parameters (in [0,1])
                       double Mohr_cohesion,  ///< cohesion for shear failure [Pa]
                       double Mohr_friction,  ///< friction angle for shear failure [degree]
                       double Janosi_shear    ///< shear parameter in Janosi-Hanamoto formula [m]
    );

  private:
    double area_ratio;     ///< fraction of contactable surface where soil-soil parameters are overridden
    double Mohr_cohesion;  ///< cohesion for shear failure [Pa]
    double Mohr_mu;        ///< coefficient of friction for shear failure [degree]
    double Janosi_shear;   ///< shear parameter in Janosi-Hanamoto formula [m]

    friend class SCMLoader;
};

/// Underlying implementation of the Soil Contact Model.
class CH_VEHICLE_API SCMLoader : public ChLoadContainer {
  public:
    SCMLoader(ChSystem* system, bool visualization_mesh);
    ~SCMLoader() {}

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double hsizeX,  ///< [in] terrain dimension in the X direction
                    double hsizeY,  ///< [in] terrain dimension in the Y direction
                    double delta    ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed mesh is provided via the specified image file as a height map.
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (image file)
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax,                        ///< [in] maximum height (white level)
                    double delta                        ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed terrain profile is provided via the specified Wavefront OBJ mesh file.
    void Initialize(const std::string& mesh_file,  ///< [in] filename for the mesh (Wavefront OBJ)
                    double delta                   ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed terrain profile is provided via the specified triangular mesh.
    void Initialize(const ChTriangleMeshConnected& trimesh,  ///< [in] surface triangular mesh
                    double delta                             ///< [in] grid spacing
    );

  private:
    // SCM patch type.
    enum class PatchType {
        FLAT,        // flat patch
        HEIGHT_MAP,  // triangular mesh (generated from a gray-scale image height-map)
        TRI_MESH     // triangular mesh (provided through an OBJ file)
    };

    // Active domain parameters.
    struct ActiveDomainInfo {
        std::shared_ptr<ChBody> m_body;   // tracked body
        ChVector3d m_center;              // OOBB center, relative to body
        ChVector3d m_hdims;               // OOBB half-dimensions
        std::vector<ChVector2i> m_range;  // current grid nodes covered by the domain
        ChVector3d m_ooN;                 // current inverse of SCM normal in body frame
    };

    /// Storage precision for the per-node soil state and the base height field.
    ///
    /// These two structures are the whole per-rank memory cost of an SCM patch: the dense
    /// m_heights matrix is (2*nx+1)*(2*ny+1) entries allocated up front whether or not a node
    /// is ever touched, and m_grid_map grows one NodeRecord per node the vehicles actually
    /// deform. At a 1024 m patch and 0.1 m spacing that is 104.9 M nodes, so the choice of
    /// scalar here is worth ~400 MB per MPI rank on its own -- and every rank of a distributed
    /// run allocates its own copy of the full grid.
    ///
    /// float is sufficient for terrain-scale work: its 24-bit mantissa resolves ~3 um at a
    /// node level of 25 m, three orders below the ~0.1 mm sinkage that matters to the Bekker
    /// and Janosi-Hanamoto terms. Levels are absolute heights in the SCM frame, so the margin
    /// scales with frame offset -- a patch placed thousands of metres from its frame origin
    /// would lose that headroom. Set this to double to restore the previous behaviour.
    using ScmReal = float;

    // Information at contacted node
    struct NodeRecord {
        ScmReal level_initial;      // initial node level (relative to SCM frame)
        ScmReal level;              // current node level (relative to SCM frame)
        ScmReal hit_level;          // ray hit level (relative to SCM frame)
        ChVector3<ScmReal> normal;  // normal of undeformed terrain (in SCM frame)
        ScmReal sinkage;            // along local normal direction
        ScmReal sinkage_plastic;    // along local normal direction
        ScmReal sinkage_elastic;    // along local normal direction
        ScmReal sigma;              // along local normal direction
        ScmReal sigma_yield;        // along local normal direction
        ScmReal kshear;             // along local tangent direction
        ScmReal tau;                // along local tangent direction
        bool erosion;               // for bulldozing
        ScmReal massremainder;      // for bulldozing
        ScmReal step_plastic_flow;  // for bulldozing

        NodeRecord() : NodeRecord(0, 0, ChVector3d(0, 0, 1)) {}
        ~NodeRecord() {}

        // Callers work in double throughout; narrowing happens here, at the storage boundary.
        NodeRecord(double init_level, double level, const ChVector3d& n)
            : level_initial(static_cast<ScmReal>(init_level)),
              level(static_cast<ScmReal>(level)),
              hit_level(static_cast<ScmReal>(1e9)),
              normal(ChVector3<ScmReal>(static_cast<ScmReal>(n.x()), static_cast<ScmReal>(n.y()),
                                        static_cast<ScmReal>(n.z()))),
              sinkage(static_cast<ScmReal>(init_level - level)),
              sinkage_plastic(0),
              sinkage_elastic(0),
              sigma(0),
              sigma_yield(0),
              kshear(0),
              tau(0),
              erosion(false),
              massremainder(0),
              step_plastic_flow(0) {}
    };

    // Hash function for a pair of integer grid coordinates
    struct CoordHash {
      public:
        // 31 is just a decently-sized prime number to reduce bucket collisions
        std::size_t operator()(const ChVector2i& p) const { return p.x() * 31 + p.y(); }
    };

    /// Sparse store for deformed-node state, tiled so that neighbouring nodes share a hash probe.
    ///
    /// WHAT THIS REPLACES AND WHY. This was a flat std::unordered_map<ChVector2i, NodeRecord>, one
    /// entry per deformed node. That is a natural fit for how the data is written -- nodes are
    /// deformed one at a time, scattered -- and a bad fit for how it is READ. The ray-cast loop
    /// asks for the current height of every node of every active domain on every step, and each
    /// ask was an independent hash probe: hash, index a bucket array, chase a node pointer. Two
    /// dependent random memory accesses, times the domain node count, times 2000 steps per
    /// simulated second.
    ///
    /// While the map stayed small that was nearly free, because the whole table sat in
    /// last-level cache. The map does not stay small. It only ever grows -- a node enters on
    /// first contact and is never evicted -- so it grows with ground covered, without bound. A
    /// measured 13.7 h two-rover run at 0.02 m spacing went from 52.8k entries (~5 MB, cached)
    /// to 891k (~85 MB, not cached) and its wall/sim climbed from 70 to 120 cumulative, which
    /// for growth this shape means the instantaneous rate roughly tripled. Nothing physical
    /// changed. The same work simply stopped hitting cache.
    ///
    /// HOW TILING FIXES IT. Nodes are grouped into kDim x kDim blocks and one hash entry holds a
    /// whole block. UpdateActiveDomain lays m_range out as [j * n_x + i], so walking it advances
    /// x fastest: kDim consecutive lookups land in the same tile, and within a tile the records
    /// are contiguous with x as the fast axis. So the probe count falls by up to kDim and, more
    /// importantly, the accesses that remain are sequential inside one 16 KB block, which
    /// hardware prefetching handles and a scattered hash map defeats. Measured on a standalone
    /// benchmark of this exact access pattern: a flat map degrades from 36.5 ns to 80-94 ns per
    /// probe as the footprint grows 50k -> 900k nodes, while this stays between 3.2 and 6.7 ns.
    /// The cliff is what the growth was; removing the cliff is what removes the growth.
    ///
    /// The cost is slack: a tile allocates all kDim*kDim records once any one of them is touched.
    /// For the footprints that matter this is close to free -- ruts and track prints are
    /// contiguous, so interior tiles are full -- and the pathological case (isolated nodes far
    /// apart) is bounded by kDim*kDim x the useful data. At 0.02 m a rut is ~50 nodes across, so
    /// kDim = 16 sits well inside it; raising kDim buys fewer probes and pays in slack.
    ///
    /// Deliberately NOT an std::unordered_map drop-in. Find() returns a pointer and there is no
    /// end(), no insert(), no operator[], so every former call site had to be revisited rather
    /// than silently keeping map semantics that no longer hold.
    class NodeMap {
      public:
        static constexpr int kShift = 4;             ///< log2 of the tile edge
        static constexpr int kDim = 1 << kShift;     ///< tile edge, in nodes
        static constexpr int kArea = kDim * kDim;    ///< records per tile

        /// Return a pointer to the record for this node, or nullptr if the node is undeformed.
        NodeRecord* Find(const ChVector2i& ij) {
            Tile* t = FindTile(ij);
            if (!t)
                return nullptr;
            const int k = IndexIn(ij);
            return t->present[k] ? &t->rec[k] : nullptr;
        }
        const NodeRecord* Find(const ChVector2i& ij) const {
            const Tile* t = FindTile(ij);
            if (!t)
                return nullptr;
            const int k = IndexIn(ij);
            return t->present[k] ? &t->rec[k] : nullptr;
        }

        /// Return the record for a node known to be present. Undefined if it is not.
        NodeRecord& at(const ChVector2i& ij) {
            NodeRecord* nr = Find(ij);
            assert(nr);
            return *nr;
        }
        const NodeRecord& at(const ChVector2i& ij) const {
            const NodeRecord* nr = Find(ij);
            assert(nr);
            return *nr;
        }

        /// Record a node if it is not already recorded, and return its record either way.
        /// Matches what std::unordered_map::insert did here: an existing record is left alone.
        NodeRecord& Emplace(const ChVector2i& ij, const NodeRecord& nr) {
            Tile* t = MakeTile(ij);
            const int k = IndexIn(ij);
            if (!t->present[k]) {
                t->rec[k] = nr;
                t->present[k] = true;
                ++t->count;
                ++m_count;
            }
            return t->rec[k];
        }

        /// Record a node, overwriting any existing record. Matches what operator[] = did here.
        NodeRecord& Set(const ChVector2i& ij, const NodeRecord& nr) {
            Tile* t = MakeTile(ij);
            const int k = IndexIn(ij);
            if (!t->present[k]) {
                t->present[k] = true;
                ++t->count;
                ++m_count;
            }
            t->rec[k] = nr;
            return t->rec[k];
        }

        /// Number of recorded (deformed) nodes.
        std::size_t size() const { return m_count; }

        /// Visit every recorded node as f(const ChVector2i& ij, const NodeRecord& nr).
        /// Visit order is unspecified and differs from the flat map's; nothing may depend on it.
        template <typename F>
        void ForEach(F&& f) const {
            for (const auto& kv : m_tiles) {
                const Tile& t = *kv.second;
                if (t.count == 0)
                    continue;
                const int base_x = kv.first.x() << kShift;
                const int base_y = kv.first.y() << kShift;
                for (int oy = 0; oy < kDim; ++oy) {
                    for (int ox = 0; ox < kDim; ++ox) {
                        const int k = (oy << kShift) + ox;
                        if (t.present[k])
                            f(ChVector2i(base_x + ox, base_y + oy), t.rec[k]);
                    }
                }
            }
        }

        void clear() {
            m_tiles.clear();
            m_count = 0;
        }

      private:
        struct Tile {
            NodeRecord rec[kArea];
            std::bitset<kArea> present;
            int count = 0;  // kept only so ForEach can skip a tile that somehow holds nothing
        };

        // Arithmetic shift is floor division, and masking with kDim-1 gives the non-negative
        // remainder, so both are correct for negative grid indices under two's complement.
        static int TileX(int v) { return v >> kShift; }
        static int Offset(int v) { return v & (kDim - 1); }
        static int IndexIn(const ChVector2i& ij) { return (Offset(ij.y()) << kShift) + Offset(ij.x()); }
        static ChVector2i KeyOf(const ChVector2i& ij) { return ChVector2i(TileX(ij.x()), TileX(ij.y())); }

        // DELIBERATELY NO LOOKUP CACHE. A one-entry "last tile" cache is the obvious next
        // optimization and it measures well in isolation (~3.9 ns/probe against ~6.7 ns without,
        // at a 900k-node footprint). It is wrong here. ComputeInternalForces casts rays under
        // `#pragma omp parallel for`, and the body of that loop calls GetHeight() -> Find() with
        // no critical section. Concurrent Find() on the flat std::unordered_map this replaced was
        // safe because it was a pure read; a mutable cache would make it a write, and two threads
        // racing on the {key, tile} pair can pair one thread's key with another's tile and return
        // a height from the wrong tile. That is silent physics corruption, not a crash. The 2.8 ns
        // is not worth it against the ~80 ns this structure already saves per probe.
        //
        // Reads stay const and share-nothing; only Emplace/Set mutate, and both are called from
        // serial code (the per-thread hit lists are merged after the parallel region closes).
        Tile* FindTile(const ChVector2i& ij) const {
            auto it = m_tiles.find(KeyOf(ij));
            return it == m_tiles.end() ? nullptr : it->second.get();
        }
        Tile* MakeTile(const ChVector2i& ij) {
            auto& slot = m_tiles[KeyOf(ij)];
            if (!slot)
                slot = std::make_unique<Tile>();
            return slot.get();
        }

        std::unordered_map<ChVector2i, std::unique_ptr<Tile>, CoordHash> m_tiles;
        std::size_t m_count = 0;
    };

    // Create visualization mesh
    void CreateVisualizationMesh(double sizeX, double sizeY);

    // Get the initial undeformed terrain height (relative to the SCM plane) at the specified grid node.
    double GetInitHeight(const ChVector2i& loc) const;

    // Get the initial undeformed terrain normal (relative to the SCM plane) at the specified grid node.
    ChVector3d GetInitNormal(const ChVector2i& loc) const;

    // Get the terrain point (relative to the SCM plane) at the specified grid node.
    ChVector3d GetPoint(const ChVector2i& loc) const;

    // Get the terrain height (relative to the SCM plane) at the specified grid node.
    double GetHeight(const ChVector2i& loc) const;

    // Get the terrain normal (relative to the SCM plane) at the specified grid node.
    ChVector3d GetNormal(const ChVector2i& loc) const;

    // Get the initial terrain height (expressed in World frame) below the specified location.
    double GetInitHeight(const ChVector3d& loc) const;

    // Get the initial terrain normal (expressed in World frame) at the point below the specified location.
    ChVector3d GetInitNormal(const ChVector3d& loc) const;

    // Get the terrain point (expressed in World frame) below the specified location.
    ChVector3d GetPoint(const ChVector3d& loc) const;

    // Get the terrain height (expressed in World frame) below the specified location.
    double GetHeight(const ChVector3d& loc) const;

    // Get the terrain normal (expressed in World frame) at the point below the specified location.
    ChVector3d GetNormal(const ChVector3d& loc) const;

    // Get index of trimesh vertex corresponding to the specified grid node.
    int GetMeshVertexIndex(const ChVector2i& loc);

    // Get indices of trimesh faces incident to the specified grid vertex.
    std::vector<std::int64_t> GetMeshFaceIndices(const ChVector2i& loc);

    // Check if the provided grid location is within the visualization mesh bounds
    bool CheckMeshBounds(const ChVector2i& loc) const;

    // Return information at node closest to specified location.
    SCMTerrain::NodeInfo GetNodeInfo(const ChVector3d& loc) const;

    // Complete setup before first simulation step.
    virtual void SetupInitial() override;

    // Update the forces and the geometry, at the beginning of each timestep.
    virtual void Setup() override {
        ComputeInternalForces();
        ChLoadContainer::Update(ChTime, UpdateFlags::UPDATE_ALL);
    }

    virtual void Update(double time, UpdateFlags update_flags) override {
        // Note!!! we cannot call ComputeInternalForces here, because Update() could
        // be called multiple times per timestep and not necessarily in time-increasing order;
        // this is a problem because in this force model the force is dissipative and keeps a 'history'.
        // Instead, we invoke ComputeInternalForces only at the beginning of the timestep in Setup().
        ChPhysicsItem::Update(time, update_flags);
    }

    // Synchronize information for a user-provided active domain.
    void UpdateActiveDomain(ActiveDomainInfo& ad, const ChVector3d& Z);

    // Synchronize information for the default active domain.
    void UpdateDefaultActiveDomain(ActiveDomainInfo& ad);

    // Ray-OBB intersection test
    bool RayOBBtest(const ActiveDomainInfo& ad, const ChVector3d& from, const ChVector3d& Z);

    // Candidate discovery shared by the CPU reference and HIP ray-cast backends (see SCMTerrain.cpp).
    void DiscoverRaycastCandidates(std::vector<ChBody*>& candidates);

    // GPU ray-cast reference backend (CPU stand-in).
    // Requires m_user_domains (explicit per-body active domains); caller checks this before invoking.
    void ComputeRayCastGpuReference(std::vector<RaycastHit>& out_hits, int& num_ray_casts);

    // Reset the list of forces and fill it with forces from the soil contact model.
    // This is called automatically during timestepping (only at the beginning of each step).
    void ComputeInternalForces();

#ifdef CHRONO_HAS_SCM_GPU
    bool ComputeContactForcesGpu(const std::unordered_map<ChVector2i, scm_gpu::ScmHitRecord, CoordHash>& hits, const std::vector<double>& patch_oob);
    scm_gpu::Config m_scm_gpu_config;

    // GPU ray-cast HIP backend. Same I/O contract as
    // ComputeRayCastGpuReference; requires m_user_domains, caller checks this before invoking.
    // Uses a process-wide singleton GPU context (scm_gpu::RaycastGpuContext()), same pattern as the
    // contact-force backend's GpuContext() in SCMTerrainGpu.cpp.
    //
    // Returns false if this backend cannot produce the step's hits -- no candidate bodies, none of
    // them carrying triangle-mesh collision geometry (the only shape type these kernels represent),
    // or a GPU call failing. The caller must then run the CPU path: reporting zero hits instead
    // would leave the terrain undeformed and the model unsupported, with no error anywhere.
    bool ComputeRayCastGpuHip(std::vector<RaycastHit>& out_hits, int& num_ray_casts);

    // On by default: if the build has the GPU backend and the model supplies active domains, existing
    // code should get the GPU without being rewritten to ask for it. The dispatch site guards on
    // m_user_domains, so a model without active domains just keeps using the CPU path.
    bool m_raycast_gpu_hip_enabled = true;
#endif

    // Override the ChLoadContainer method for computing the generalized force F term:
    virtual void IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                   ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                   const double c           // a scaling factor
                                   ) override {
        ChLoadContainer::IntLoadResidual_F(off, R, c);
    }

    // Add specified amount of material (possibly clamped) to node.
    void AddMaterialToNode(double amount, NodeRecord& nr);

    // Remove specified amount of material (possibly clamped) from node.
    void RemoveMaterialFromNode(double amount, NodeRecord& nr);

    // Update vertex position and color in visualization mesh
    void UpdateMeshVertexCoordinates(const ChVector2i ij, int iv, const NodeRecord& nr);

    // Update vertex normal in visualization mesh
    void UpdateMeshVertexNormal(const ChVector2i ij, int iv);

    /// Get the heights of all modified grid nodes.
    /// If 'all_nodes = true', return modified nodes from the start of simulation.  Otherwise, return only the nodes
    /// modified over the last step.
    std::vector<SCMTerrain::NodeLevel> GetModifiedNodes(bool all_nodes = false) const;

    // Modify the level of grid nodes from the given list.
    void SetModifiedNodes(const std::vector<SCMTerrain::NodeLevel>& nodes);

    PatchType m_type;      ///< type of SCM patch
    ChCoordsys<> m_frame;  ///< SCM frame (deformation occurs along the z axis of this frame)
    ChVector3d m_Z;        ///< SCM plane vertical direction (in absolute frame)
    double m_delta;        ///< grid spacing
    double m_area;         ///< area of a grid cell
    int m_nx;              ///< range for grid indices in X direction: [-m_nx, +m_nx]
    int m_ny;              ///< range for grid indices in Y direction: [-m_ny, +m_ny]

    ChMatrixDynamic<ScmReal> m_heights;  ///< (base) grid heights (when initializing from height-field map)
    double m_base_height;         ///< default height for vertices outside the projection of input mesh

    NodeMap m_grid_map;  ///< deformed grid nodes (persistent, tiled; see NodeMap)
    std::vector<ChVector2i> m_modified_nodes;                          ///< modified grid nodes (current)

    ChAABB m_aabb;    ///< user-specified SCM terrain boundary
    bool m_boundary;  ///< user-specified SCM terrain boundary?

    std::vector<ActiveDomainInfo> m_active_domains;  ///< set of active domains
    bool m_user_domains;                             ///< user-specified active domains?

    double m_test_offset_down;  ///< offset for ray start
    double m_test_offset_up;    ///< offset for ray end

    int m_num_raycast_gpu_steps = 0;        ///< steps whose ray casting was actually done on the GPU
    int m_num_contact_force_gpu_steps = 0;  ///< steps whose contact forces were actually done on the GPU
    bool m_raycast_gpu_ref_enabled;         ///< use the GPU ray-cast reference backend

    std::shared_ptr<ChVisualShapeTriangleMesh> m_trimesh_shape;  ///< mesh visualization asset
    std::unique_ptr<ChColormap> m_colormap;                      ///< colormap for mesh false coloring
    ChColormap::Type m_colormap_type;                            ///< colormap type

    bool m_cosim_mode;  ///< co-simulation mode

    // SCM parameters
    double m_Bekker_Kphi;    ///< frictional modulus in Bekker model
    double m_Bekker_Kc;      ///< cohesive modulus in Bekker model
    double m_Bekker_n;       ///< exponent of sinkage in Bekker model (usually 0.6...1.8)
    double m_Mohr_cohesion;  ///< cohesion for shear failure [Pa]
    double m_Mohr_mu;        ///< coefficient of friction for shear failure [degree]
    double m_Janosi_shear;   ///< shear parameter in Janosi-Hanamoto formula [m]
    double m_elastic_K;      ///< elastic stiffness K per unit area, [Pa/m] (must be larger than Kphi)
    double m_damping_R;      ///< vertical damping R per unit area [Pa.s/m] (proportional to vertical speed)

    // Callback object for position-dependent soil properties
    std::shared_ptr<SCMTerrain::SoilParametersCallback> m_soil_fun;

    // Contact forces on contactable objects interacting with the SCM terrain
    std::unordered_map<ChBody*, std::pair<ChVector3d, ChVector3d>> m_body_forces;
#ifdef CHRONO_FEA
    std::unordered_map<std::shared_ptr<fea::ChNodeFEAxyz>, ChVector3d> m_node_forces;
#endif

    // Bulldozing effects
    bool m_bulldozing;
    double m_flow_factor;
    double m_erosion_slope;
    int m_erosion_iterations;
    int m_erosion_propagations;

    // Mesh coloring mode
    SCMTerrain::DataPlotType m_plot_type;
    double m_plot_v_min;
    double m_plot_v_max;

    // Indices of visualization mesh vertices modified externally
    std::vector<int> m_external_modified_vertices;

    // Timers and counters
    ChTimer m_timer_active_domains;
    ChTimer m_timer_ray_testing;
    ChTimer m_timer_ray_casting;
    ChTimer m_timer_contact_patches;
    ChTimer m_timer_contact_forces;
    ChTimer m_timer_bulldozing;
    ChTimer m_timer_bulldozing_boundary;
    ChTimer m_timer_bulldozing_domain;
    ChTimer m_timer_bulldozing_erosion;
    ChTimer m_timer_visualization;
    int m_num_ray_casts;
    int m_num_ray_hits;
    int m_num_contact_patches;
    int m_num_erosion_nodes;

    friend class SCMTerrain;
    friend class ChScmVisualizationVSG;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
