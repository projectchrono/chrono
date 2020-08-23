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

#ifndef SCM_DEFORMABLE_TERRAIN_H
#define SCM_DEFORMABLE_TERRAIN_H

#include <set>
#include <string>
#include <unordered_map>
#include <ostream>

#include "chrono/assets/ChColorAsset.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/core/ChTimer.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/ChWorldFrame.h"

namespace chrono {
namespace vehicle {

class SCMDeformableSoil;

/// @addtogroup vehicle_terrain
/// @{

/// Deformable terrain model.
/// This class implements a deformable terrain based on the Soil Contact Model.
/// Unlike RigidTerrain, the vertical coordinates of this terrain mesh can be deformed
/// due to interaction with ground vehicles or other collision shapes.
class CH_VEHICLE_API SCMDeformableTerrain : public ChTerrain {
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
        PLOT_PRESSURE_YELD,
        PLOT_SHEAR,
        PLOT_K_JANOSI,
        PLOT_IS_TOUCHED,
        PLOT_ISLAND_ID,
        PLOT_MASSREMAINDER
    };

    /// Construct a default SCM deformable terrain.
    /// The user is responsible for calling various Set methods before Initialize.
    SCMDeformableTerrain(ChSystem* system,               ///< [in] pointer to the containing multibody system
                         bool visualization_mesh = true  ///< [in] enable/disable visualization asset
    );

    ~SCMDeformableTerrain() {}

    /// Set the plane reference.
    /// By default, the reference plane is horizontal with Z up (ISO vehicle reference frame).
    /// To set as Y up, call SetPlane(ChCoordys(VNULL, Q_from_AngX(-CH_C_PI_2)));
    void SetPlane(const ChCoordsys<>& plane);

    /// Get the current reference plane.
    /// The SCM terrain patch is in the (x,y) plane with normal along the Z axis.
    const ChCoordsys<>& GetPlane() const;

    /// Set the properties of the SCM soil model.
    /// These parameters are described in: "Parameter Identification of a Planetary Rover Wheel-Soil Contact Model via a
    /// Bayesian Approach", A.Gallina, R. Krenn et al. Note that the original SCM model does not include the K and R
    /// coefficients. A very large value of K and R=0 reproduce the original SCM.
    void SetSoilParameters(
        double Bekker_Kphi,    ///< Kphi, frictional modulus in Bekker model
        double Bekker_Kc,      ///< Kc, cohesive modulus in Bekker model
        double Bekker_n,       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        double Mohr_cohesion,  ///< Cohesion in, Pa, for shear failure
        double Mohr_friction,  ///< Friction angle (in degrees!), for shear failure
        double Janosi_shear,   ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
        double elastic_K,      ///< elastic stiffness K, per unit area, [Pa/m] (must be larger than Kphi)
        double damping_R       ///< vertical damping R, per unit area [Pa s/m] (proportional to vertical speed)
    );

    /// Enable/disable the creation of soil inflation at the side of the ruts (bulldozing effects).
    void SetBulldozingFlow(bool mb);

    /// Return true if bulldozing effects are enabled.
    bool BulldozingFlow() const;

    /// Set parameters controlling the creation of side ruts (bulldozing effects).
    void SetBulldozingParameters(
        double mbulldozing_erosion_angle,            ///< angle of erosion of the displaced material (in degrees!)
        double mbulldozing_flow_factor = 1.0,        ///< growth of lateral volume respect to pressed volume
        int mbulldozing_erosion_n_iterations = 3,    ///< number of erosion refinements per timestep
        int mbulldozing_erosion_n_propagations = 10  ///< number of concentric vertex selections subject to erosion
    );

    /// Set the vertical level up to which collision is tested (relative to the reference level at the sample point).
    /// Since the contact is unilateral, this could be zero. However, when computing bulldozing flow, one might also
    /// need to know if in the surrounding there is some potential future contact: so it might be better to use a
    /// positive value (but not higher than the max. expected height of the bulldozed rubble, to avoid slowdown of
    /// collision tests).
    void SetTestHeight(double offset);

    ///  Return the current test height level.
    double GetTestHeight() const;

    /// Set the color plot type for the soil mesh.
    /// When a scalar plot is used, also define the range in the pseudo-color colormap.
    void SetPlotType(DataPlotType plot_type, double min_val, double max_val);

    /// Set visualization color.
    void SetColor(const ChColor& color);

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float tex_scale_x = 1,       ///< [in] texture scale in X
                    float tex_scale_y = 1        ///< [in] texture scale in Y
    );

    /// Add a new moving patch.
    /// Multiple calls to this function can be made, each of them adding a new active patch area.
    /// If no patches are defined, ray-casting is performed for every single node of the underlying SCM grid.
    /// If at least one patch is defined, ray-casting is performed only for mesh nodes within the AABB of the
    /// body OOBB projection onto the SCM plane.
    void AddMovingPatch(std::shared_ptr<ChBody> body,   ///< [in] monitored body
                        const ChVector<>& OOBB_center,  ///< [in] OOBB center, relative to body
                        const ChVector<>& OOBB_dims     ///< [in] OOBB dimensions
    );

    /// Class to be used as a callback interface for location-dependent soil parameters.
    /// A derived class must implement Set() and set **all** soil parameters (no defaults are provided).
    class CH_VEHICLE_API SoilParametersCallback {
      public:
        virtual ~SoilParametersCallback() {}

        /// Set the soil properties at a given (x,y) location.
        /// Attention: the location is assumed to be provided in the (x,y) plane of the patch reference plane!
        /// An implementation of this method in a derived class must set all soil parameters.
        virtual void Set(double x, double y) = 0;

        double m_Bekker_Kphi;    ///< Kphi, frictional modulus in Bekker model
        double m_Bekker_Kc;      ///< Kc, cohesive modulus in Bekker model
        double m_Bekker_n;       ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
        double m_Mohr_cohesion;  ///< Cohesion in, Pa, for shear failure
        double m_Mohr_friction;  ///< Friction angle (in degrees!), for shear failure
        double m_Janosi_shear;   ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
        double m_elastic_K;      ///< elastic stiffness K, per unit area, [Pa/m] (must be larger than Kphi)
        double m_damping_R;      ///< vertical damping R, per unit area [Pa s/m] (proportional to vertical speed)
    };

    /// Specify the callback object to set the soil parameters at given (x,y) locations.
    /// To use constant soil parameters throughout the entire patch, use SetSoilParameters.
    void RegisterSoilParametersCallback(std::shared_ptr<SoilParametersCallback> cb);

    /// Get the terrain height below the specified location.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For SCMDeformableTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value of 0.8.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

    /// Get the visualization triangular mesh.
    std::shared_ptr<ChTriangleMeshShape> GetMesh() const;

    /// Save the visualization mesh as a Wavefront OBJ file.
    void WriteMesh(const std::string& filename) const;

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double sizeX,  ///< [in] terrain dimension in the X direction
                    double sizeY,  ///< [in] terrain dimension in the Y direction
                    double delta   ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed mesh is provided via the specified image file as a height map.
    /// By default, a mesh vertex is created for each pixel in the image. If divX and divY are non-zero,
    /// the image will be sampled at the specified resolution with linear interpolation as needed.
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (image file)
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax,                        ///< [in] maximum height (white level)
                    double delta                        ///< [in] grid spacing (may be slightly decreased)
    );

    /// Vertex height level at a given grid location.
    typedef std::pair<ChVector2<int>, double> VertexLevel;

    /// Get the grid vertices and their heights that were modified over last step.
    std::vector<VertexLevel> GetModifiedVertices() const;

    /// Modify the level of vertices in the underlying grid map from the given list.
    void SetModifiedVertices(const std::vector<VertexLevel>& vertices);

    /// Return the current cumulative contact force on the specified body (due to interaction with the SCM terrain).
    TerrainForce GetContactForce(std::shared_ptr<ChBody> body) const;

    /// Print timing and counter information for last step.
    void PrintStepStatistics(std::ostream& os) const;

  private:
    std::shared_ptr<SCMDeformableSoil> m_ground;  ///< undelrying load container for contact force generation
};

/// This class provides the underlying implementation of the Soil Contact Model.
/// Used in SCMDeformableTerrain.
class CH_VEHICLE_API SCMDeformableSoil : public ChLoadContainer {
  public:
    SCMDeformableSoil(ChSystem* system, bool visualization_mesh);
    ~SCMDeformableSoil() {}

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double hsizeX,  ///< [in] terrain dimension in the X direction
                    double hsizeY,  ///< [in] terrain dimension in the Y direction
                    double delta    ///< [in] grid spacing (may be slightly decreased)
    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed mesh is provided via the specified image file as a height map
    /// By default, a mesh vertex is created for each pixel in the image. If divX and divY are non-zero,
    /// the image will be sampled at the specified resolution with linear interpolation as needed.
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (image file)
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax,                        ///< [in] maximum height (white level)
                    double delta                        ///< [in] grid spacing (may be slightly decreased)
    );

  private:
    // SCM patch type
    enum class PatchType {
        FLAT,       // flat patch
        HEIGHT_MAP  // triangular mesh (generated from a gray-scale image height-map)
    };

    // Moving patch parameters
    struct MovingPatchInfo {
        std::shared_ptr<ChBody> m_body;  // tracked body
        ChVector<> m_center;             // OOBB center, relative to body
        ChVector<> m_hdims;              // OOBB half-dimensions
        ChVector2<int> m_bl;             // current grid coords of bottom-left corner
        ChVector2<int> m_tr;             // current grid coords of top-right corner
        ChVector<> m_ooN;                // current inverse of SCM normal in body frame
    };

    // Information of vertices with ray-cast hits
    struct HitRecord {
        ChContactable* contactable;  // pointer to hit object
        ChVector<> abs_point;        // hit point, expressed in global frame
        int patch_id;                // index of associated patch id
    };

    // Information at contacted vertex
    struct VertexRecord {
        double p_sigma;
        double p_sinkage_elastic;
        double p_step_plastic_flow;
        bool p_erosion;
        double p_level;
        double p_hit_level;
        double p_level_initial;
        ChVector<> p_speeds;
        double p_sinkage_plastic;
        double p_sinkage;
        double p_kshear;
        double p_sigma_yield;
        double p_tau;

        VertexRecord() : VertexRecord(0, 0) {}

        VertexRecord(double init_level, double level) {
            p_sigma = 0;
            p_sinkage_elastic = 0;
            p_sinkage_plastic = 0;
            p_step_plastic_flow = 0;
            p_erosion = false;
            p_level = level;
            p_level_initial = init_level;
            p_hit_level = 1e9;
            p_sinkage_plastic = 0;
            p_sinkage = 0;
            p_kshear = 0;
            p_sigma_yield = 0;
            p_tau = 0;
        }
    };

    // Hash function for a pair of integer grid coordinates
    struct CoordHash {
      public:
        // 31 is just a decently-sized prime number to reduce bucket collisions
        std::size_t operator()(const ChVector2<int>& p) const { return p.x() * 31 + p.y(); }
    };

    // Get the terrain normal at the point below the specified location.
    ChVector<> GetNormal(const ChVector<>& loc) const;

    // Get the terrain height below the specified location.
    double GetHeight(const ChVector<>& loc) const;

    // Get the terrain height (relative to the SCM plane) at the specified grid vertex.
    double GetHeight(const ChVector2<int>& loc) const;

    // Get index of trimesh vertex corresponding to the specified grid vertex.
    int GetMeshVertexIndex(const ChVector2<int>& loc);

    // Get indices of trimesh faces incident to the specified grid vertex.
    std::vector<int> GetMeshFaceIndices(const ChVector2<int>& loc);

    // Complete setup before first simulation step.
    virtual void SetupInitial() override;

    // Updates the forces and the geometry, at the beginning of each timestep
    virtual void Setup() override {
        ComputeInternalForces();
        ChLoadContainer::Update(ChTime, true);
    }

    // Updates the forces and the geometry
    virtual void Update(double mytime, bool update_assets = true) override {
        // Note!!! we cannot call ComputeInternalForces here, because Update() could
        // be called multiple times per timestep and not necessarily in time-increasing order;
        // this is a problem because in this force model the force is dissipative and keeps a 'history'.
        // Instead, we invoke ComputeInternalForces only at the beginning of the timestep in Setup().

        ChTime = mytime;
    }

    // Synchronize information for a moving patch
    void UpdateMovingPatch(MovingPatchInfo& p, const ChVector<>& N);

    // Synchronize information for fixed patch
    void UpdateFixedPatch(MovingPatchInfo& p);

    // Ray-OBB intersection test
    bool RayOBBtest(const MovingPatchInfo& p, const ChVector<>& from, const ChVector<>& N);

    // Reset the list of forces and fill it with forces from the soil contact model.
    // This is called automatically during timestepping (only at the beginning of each step).
    void ComputeInternalForces();

    // Override the ChLoadContainer method for computing the generalized force F term:
    virtual void IntLoadResidual_F(const unsigned int off,  // offset in R residual
                                   ChVectorDynamic<>& R,    // result: the R residual, R += c*F
                                   const double c           // a scaling factor
                                   ) override {
        ChLoadContainer::IntLoadResidual_F(off, R, c);
    }

    // Update vertex position and color in visualization mesh
    void UpdateMeshVertexCoordinates(const ChVector2<int> ij, int iv, const VertexRecord& v);

    // Update vertex normal in visualization mesh
    void UpdateMeshVertexNormal(const ChVector2<int> ij, int iv);

    /// Get the grid vertices and their heights that were modified over last step.
    std::vector<SCMDeformableTerrain::VertexLevel> GetModifiedVertices() const;

    // Modify the level of vertices in the underlying grid map from the given list.
    void SetModifiedVertices(const std::vector<SCMDeformableTerrain::VertexLevel>& vertices);

    PatchType m_type;      // type of SCM patch
    ChCoordsys<> m_plane;  // SCM frame (deformation occurs along the z axis of this frame)
    double m_delta;        // (base) grid spacing
    double m_area;         // area of a (base) grid cell
    int m_nx;              // range for grid indices in X direction: [-m_nx, +m_nx]
    int m_ny;              // range for grid indices in Y direction: [-m_ny, +m_ny]

    ChMatrixDynamic<> m_heights;  // (base) grid heights (when initializing from height-field map)

    std::unordered_map<ChVector2<int>, HitRecord, CoordHash> m_hits;         // hash-map for vertices with ray-cast hits
    std::unordered_map<ChVector2<int>, VertexRecord, CoordHash> m_grid_map;  // hash-map for modified vertices

    std::vector<MovingPatchInfo> m_patches;  // set of active moving patches
    bool m_moving_patch;                     // user-specified moving patches?

    double m_test_offset_down;  // offset for ray start
    double m_test_offset_up;    // offset for ray end

    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;  // mesh visualization asset
    std::shared_ptr<ChColorAsset> m_color;                 // mesh edge default color

    // SCM parameters
    double m_Bekker_Kphi;
    double m_Bekker_Kc;
    double m_Bekker_n;
    double m_Mohr_cohesion;
    double m_Mohr_friction;
    double m_Janosi_shear;
    double m_elastic_K;
    double m_damping_R;

    // Callback object for position-dependent soil properties
    std::shared_ptr<SCMDeformableTerrain::SoilParametersCallback> m_soil_fun;

    // Contact forces on contactable objects interacting with the SCM terrain
    std::unordered_map<ChContactable*, TerrainForce> m_contact_forces;

    // Bulldozing effects
    bool do_bulldozing;
    double bulldozing_flow_factor;
    double bulldozing_erosion_angle;
    int bulldozing_erosion_n_iterations;
    int bulldozing_erosion_n_propagations;

    // Mesh coloring mode
    SCMDeformableTerrain::DataPlotType m_plot_type;
    double m_plot_v_min;
    double m_plot_v_max;

    // Indices of visualization mesh vertices modified externally
    std::vector<int> m_external_modified_vertices;

    // Timers and counters
    ChTimer<double> m_timer_moving_patches;
    ChTimer<double> m_timer_ray_casting;
    ChTimer<double> m_timer_contact_patches;
    ChTimer<double> m_timer_contact_forces;
    ChTimer<double> m_timer_bulldozing;
    ChTimer<double> m_timer_visualization;
    int m_num_ray_casts;
    int m_num_ray_hits;
    int m_num_contact_patches;

    friend class SCMDeformableTerrain;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
