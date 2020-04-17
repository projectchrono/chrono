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
// Authors: Alessandro Tasora, Radu Serban
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

    /// Construct a default SCMDeformableSoil.
    /// The user is responsible for calling various Set methods before Initialize.
    SCMDeformableTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
    );

    ~SCMDeformableTerrain() {}

    /// Set the plane reference.
    /// By default, the reference plane is horizontal with Z up (ISO vehicle reference frame).
    /// To set as Y up, call SetPlane(ChCoordys(VNULL, Q_from_AngX(-CH_C_PI_2)));
    void SetPlane(ChCoordsys<> mplane);

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
    bool GetBulldozingFlow() const;

    /// Set parameters controlling the creation of side ruts (bulldozing effects).
    void SetBulldozingParameters(
        double mbulldozing_erosion_angle,            ///< angle of erosion of the displaced material (in degrees!)
        double mbulldozing_flow_factor = 1.0,        ///< growth of lateral volume respect to pressed volume
        int mbulldozing_erosion_n_iterations = 3,    ///< number of erosion refinements per timestep
        int mbulldozing_erosion_n_propagations = 10  ///< number of concentric vertex selections subject to erosion
    );

    /// Enable/disable dynamic mesh refinement.
    /// If enabled, additional points are added under contact patches, up to the desired resolution.
    void SetAutomaticRefinement(bool mr);
    bool GetAutomaticRefinement() const;

    /// Set the resolution for automatic mesh refinement (specified in meters).
    /// The mesh is refined, as needed, until the largest side of a triangle reaches trhe specified resolution.
    /// Triangles out of contact patches are not refined.
    void SetAutomaticRefinementResolution(double mr);
    double GetAutomaticRefinementResolution() const;

    /// Set the vertical level up to which collision is tested (relative to the reference level at the sample point).
    /// Since the contact is unilateral, this could be zero. However, when computing bulldozing flow, one might also
    /// need to know if in the surrounding there is some potential future contact: so it might be better to use a
    /// positive value (but not higher than the max. expected height of the bulldozed rubble, to avoid slowdown of
    /// collision tests).
    void SetTestHighOffset(double moff);
    double GetTestHighOffset() const;

    /// Set the color plot type for the soil mesh.
    /// When a scalar plot is used, also define the range in the pseudo-color colormap.
    void SetPlotType(DataPlotType mplot, double mmin, double mmax);

    /// Set visualization color.
    void SetColor(ChColor color  ///< [in] color of the visualization material
    );

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float tex_scale_x = 1,       ///< [in] texture scale in X
                    float tex_scale_y = 1        ///< [in] texture scale in Y
    );

    /// Add a new moving patch.
    /// Multiple calls to this function can be made, each of them adding a new active patch area.
    /// If no patches are defined, ray-casting is performed for every single node of the underlying SCM mesh.
    /// If at least one patch is defined, ray-casting is performed only for mesh nodes within the patch areas
    /// (that is, nodes that are within the specified range from the given point on the associated body).
    void AddMovingPatch(std::shared_ptr<ChBody> body,     ///< [in] monitored body
                        const ChVector<>& point_on_body,  ///< [in] patch center, relative to body
                        double dimX,                      ///< [in] patch X dimension
                        double dimY                       ///< [in] patch Y dimension
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
    void RegisterSoilParametersCallback(SoilParametersCallback* cb);

    /// Get the terrain height below the specified location.
    virtual double GetHeight(const ChVector<>& loc) const override;

    /// Get the terrain normal at the point below the specified location.
    virtual chrono::ChVector<> GetNormal(const ChVector<>& loc) const override;

    /// Get the terrain coefficient of friction at the point below the specified location.
    /// This coefficient of friction value may be used by certain tire models to modify
    /// the tire characteristics, but it will have no effect on the interaction of the terrain
    /// with other objects (including tire models that do not explicitly use it).
    /// For SCMDeformableTerrain, this function defers to the user-provided functor object
    /// of type ChTerrain::FrictionFunctor, if one was specified.
    /// Otherwise, it returns the constant value of 0.8.
    virtual float GetCoefficientFriction(const ChVector<>& loc) const override;

    /// Get the current reference plane. The SCM terrain patch is in the (x,y) plane with normal along the Z axis.
    const ChCoordsys<>& GetPlane() const;

    /// Get the underlyinh triangular mesh.
    const std::shared_ptr<ChTriangleMeshShape> GetMesh() const;

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double height,  ///< [in] terrain height
                    double sizeX,   ///< [in] terrain dimension in the X direction
                    double sizeY,   ///< [in] terrain dimension in the Y direction
                    int divX,       ///< [in] number of divisions in the X direction
                    int divY        ///< [in] number of divisions in the Y direction
    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed mesh is provided via a Wavefront .obj file.
    void Initialize(const std::string& mesh_file  ///< [in] filename of the input mesh (.OBJ file in Wavefront format)
    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed mesh is provided via the specified BMP file as a height map
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (BMP)
                    const std::string& mesh_name,       ///< [in] name of the mesh asset
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax                         ///< [in] maximum height (white level)
    );

    /// Return the current cumulative contact force on the specified body (due to interaction with the SCM terrain).
    TerrainForce GetContactForce(std::shared_ptr<ChBody> body) const;

    /// Print timing and counter information for last step.
    void PrintStepStatistics(std::ostream& os) const;

  private:
    std::shared_ptr<SCMDeformableSoil> m_ground;
};

/// This class provides the underlying implementation of the Soil Contact Model.
/// Used in SCMDeformableTerrain.
class CH_VEHICLE_API SCMDeformableSoil : public ChLoadContainer {
  public:
    SCMDeformableSoil(ChSystem* system);
    ~SCMDeformableSoil() {}

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double height,  ///< [in] terrain height
                    double sizeX,   ///< [in] terrain dimension in the X direction
                    double sizeY,   ///< [in] terrain dimension in the Y direction
                    int divX,       ///< [in] number of divisions in the X direction
                    int divY        ///< [in] number of divisions in the Y direction
    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed mesh is provided via a Wavefront .obj file.
    void Initialize(const std::string& mesh_file  ///< [in] filename of the input mesh (.OBJ file in Wavefront format)
    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed mesh is provided via the specified BMP file as a height map
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (BMP)
                    const std::string& mesh_name,       ///< [in] name of the mesh asset
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax                         ///< [in] maximum height (white level)
    );

  private:
    // Updates the forces and the geometry, at the beginning of each timestep
    virtual void Setup() override {
        // GetLog() << " Setup update soil t= "<< this->ChTime << "\n";
        this->ComputeInternalForces();

        ChLoadContainer::Update(ChTime, true);
    }

    // Updates the forces and the geometry
    virtual void Update(double mytime, bool update_assets = true) override {
        // Note!!! we cannot call ComputeInternalForces here, because Update() could
        // be called multiple times per timestep (ex. see HHT or RungKutta) and not
        // necessarily in time-increasing order; this is a problem because in this
        // force model the force is dissipative and keeps an 'history'. So we do
        // ComputeInternalForces only at the beginning of the timestep; look Setup().

        ChTime = mytime;
    }

    // Reset the list of forces, and fills it with forces from a soil contact model.
    // This is called automatically during timestepping (only at the beginning of
    // each IntLoadResidual_F() for performance reason, not at each Update() that might be overkill).
    void ComputeInternalForces();

    // Override the ChLoadContainer method for computing the generalized force F term:
    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ) override {
        // Overloading base class, that takes all F vectors from the list of forces and put all them in R
        ChLoadContainer::IntLoadResidual_F(off, R, c);
    }

    // This is called after Initialize(), it pre-computes aux.topology
    // data structures for the mesh, aux. material data, etc.
    void SetupAuxData();

    std::shared_ptr<ChColorAsset> m_color;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
    double m_height;

    std::vector<ChVector<>> p_vertices_initial;
    std::vector<ChVector<>> p_speeds;
    std::vector<double> p_level;
    std::vector<double> p_level_initial;
    std::vector<double> p_hit_level;
    std::vector<double> p_sinkage;
    std::vector<double> p_sinkage_plastic;
    std::vector<double> p_sinkage_elastic;
    std::vector<double> p_step_plastic_flow;
    std::vector<double> p_kshear;  // Janosi-Hanamoto shear accumulator
    std::vector<double> p_area;
    std::vector<double> p_sigma;
    std::vector<double> p_sigma_yeld;
    std::vector<double> p_tau;
    std::vector<double> p_massremainder;
    std::vector<int> p_id_island;
    std::vector<bool> p_erosion;

    double m_Bekker_Kphi;
    double m_Bekker_Kc;
    double m_Bekker_n;
    double m_Mohr_cohesion;
    double m_Mohr_friction;
    double m_Janosi_shear;
    double m_elastic_K;
    double m_damping_R;

    int plot_type;
    double plot_v_min;
    double plot_v_max;

    ChCoordsys<> plane;

    // aux. topology data
    std::vector<std::set<int>> connected_vertexes;
    std::vector<std::array<int, 4>> tri_map;

    bool do_bulldozing;
    double bulldozing_flow_factor;
    double bulldozing_erosion_angle;
    int bulldozing_erosion_n_iterations;
    int bulldozing_erosion_n_propagations;

    bool do_refinement;
    double refinement_resolution;

    double test_high_offset;
    double test_low_offset;

    double last_t;  // for optimization

    // Moving patch parameters
    struct MovingPatchInfo {
        std::shared_ptr<ChBody> m_body;  // tracked body
        ChVector<> m_point;              // patch center, relative to body
        ChVector2<> m_dim;               // patch dimensions (X,Y)
        ChVector2<> m_min;               // current patch AABB (min x,y)
        ChVector2<> m_max;               // current patch AABB (max x,y)
    };
    std::vector<MovingPatchInfo> m_patches;  // set of active moving patches
    bool m_moving_patch;                     // moving patch feature enabled?

    // Callback object for position-dependent soil properties
    SCMDeformableTerrain::SoilParametersCallback* m_soil_fun;

    // Timers and counters
    ChTimer<double> m_timer_calc_areas;
    ChTimer<double> m_timer_ray_casting;
    ChTimer<double> m_timer_refinement;
    ChTimer<double> m_timer_bulldozing;
    ChTimer<double> m_timer_visualization;
    size_t m_num_vertices;
    size_t m_num_faces;
    size_t m_num_ray_casts;
    size_t m_num_marked_faces;

    std::unordered_map<ChContactable*, TerrainForce> m_contact_forces;

    friend class SCMDeformableTerrain;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
