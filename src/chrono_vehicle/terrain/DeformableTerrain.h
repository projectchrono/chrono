// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================
//
// Deformable terrain
//
// =============================================================================

#ifndef DEFORMABLE_TERRAIN_H
#define DEFORMABLE_TERRAIN_H

#include <string>
#include <set>

#include "chrono/assets/ChColor.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono/geometry/ChCTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLoadsBody.h"

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_terrain
/// @{

/// Deformable terrain model.
/// This class implements a terrain with variable heightmap, but differently from
/// RigidTerrain, the vertical coordinates of this terrain mesh can be deformed
/// because of vertical interaction with wheeled vehicles

class CH_VEHICLE_API DeformableTerrain : public ChLoadContainer {
  public:

    /// Construct a default DeformableTerrain.
    /// The user is responsible for calling various Set methods before Initialize.
    DeformableTerrain(ChSystem* system  ///< [in] pointer to the containing multibody system
                 );

    ~DeformableTerrain() {}


    /// Set visualization color.
    void SetColor(ChColor color  ///< [in] color of the visualization material
                  );

    /// Set texture properties.
    void SetTexture(const std::string tex_file,  ///< [in] texture filename
                    float tex_scale_x = 1,       ///< [in] texture scale in X
                    float tex_scale_y = 1        ///< [in] texture scale in Y
                    );

    /// Set the plane reference. 
    /// The soil height is on the Y axis of this plane, and X-Z axes of the coordsys are the
    /// longitude-latitude. To set as Z up, do SetPlane(ChCoordys(VNULL, Q_from_AngAxis(VECT_X,-CH_C_PI_2)));
    void SetPlane(ChCoordsys<> mplane) {plane = mplane;}

    /// Get the plane reference. 
    /// The soil height is on the Y axis of this plane, and X-Z axes of the coordsys are the
    /// longitude-latitude.
    ChCoordsys<> GetPlane() {return plane;}

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(double height,  ///< [in] terrain height
                    double sizeX,   ///< [in] terrain dimension in the X direction
                    double sizeY,   ///< [in] terrain dimension in the Y direction
                    int divX,       ///< [in] number of divisions in the X direction
                    int divY       ///< [in] number of divisions in the Y direction
                    );

    /// Initialize the terrain system (mesh).
    /// The initial undeformed mesh is provided via a Wavefront .obj file. 
    void Initialize(const std::string& mesh_file  ///< [in] filename of the input mesh (.OBJ file in Wavefront format)
                    );

    /// Initialize the terrain system (height map).
    /// The initial undeformed mesh is provided via the specified BMP file as 
    /// a height map
    void Initialize(const std::string& heightmap_file,  ///< [in] filename for the height map (BMP)
                    const std::string& mesh_name,       ///< [in] name of the mesh asset
                    double sizeX,                       ///< [in] terrain dimension in the X direction
                    double sizeY,                       ///< [in] terrain dimension in the Y direction
                    double hMin,                        ///< [in] minimum height (black level)
                    double hMax                         ///< [in] maximum height (white level)
                    );

    /// Set the properties of the SCM soild model. 
    /// The meaning of these parameters is described in the paper:
    // "Parameter Identification of a Planetary Rover Wheel–Soil
    // Contact Model via a Bayesian Approach", A.Gallina, R. Krenn et al.
    void SetSoilParametersSCM( 
                    double mBekker_Kphi, ///< Kphi, frictional modulus in Bekker model 
                    double mBekker_Kc,   ///< Kc, cohesive modulus in Bekker model
                    double mBekker_n,    ///< n, exponent of sinkage in Bekker model (usually 0.6...1.8)
                    double mMohr_cohesion, ///< Cohesion in, Pa, for shear failure
                    double mMohr_friction, ///< Friction angle (in degrees!), for shear failure
                    double mJanosi_shear,  ///< J , shear parameter, in meters, in Janosi-Hanamoto formula (usually few mm or cm)
                    double melastic_K      ///< elastic stiffness K (for very high values it tends to the original SCM model)
                    ) {
        Bekker_Kphi = mBekker_Kphi;
        Bekker_Kc = mBekker_Kc;
        Bekker_n = mBekker_n;
        Mohr_cohesion = mMohr_cohesion;
        Mohr_friction = mMohr_friction;
        Janosi_shear  = mJanosi_shear;
        elastic_K  = melastic_K;
    }


    enum eChSoilDataPlot {
        PLOT_NONE ,
        PLOT_SINKAGE ,
        PLOT_SINKAGE_ELASTIC ,
        PLOT_SINKAGE_PLASTIC ,
        PLOT_STEP_PLASTIC_FLOW,
        PLOT_PRESSURE,
        PLOT_PRESSURE_YELD,
        PLOT_SHEAR,
        PLOT_K_JANOSI,
        PLOT_IS_TOUCHED,
        PLOT_ISLAND_ID
    };

    /// Set the color plot type for the soil mesh. 
    /// Also, when a scalar plot is used, also define which is the max-min range in the falsecolor colormap.
    void SetPlotType(eChSoilDataPlot mplot, double mmin, double mmax) { plot_type = mplot; plot_v_min = mmin; plot_v_max = mmax;}

    /// Get the terrain height at the specified (x,y) location.
    //virtual double GetHeight(double x, double y) const override;

    /// Get the terrain normal at the specified (x,y) location.
    //virtual chrono::ChVector<> GetNormal(double x, double y) const override;

    // Updates the forces and the geometry
    virtual void Update(double mytime, bool update_assets = true) override {
        // Computes the internal forces
        this->UpdateInternalForces();
        // Overloading base class
        ChLoadContainer::Update(mytime, update_assets);
    }

    // Reset the list of forces, and fills it with forces from a soil contact model.
    // This is called automatically during timestepping (only at the beginning of 
    // each IntLoadResidual_F() for performance reason, not at each Update() that might be overkill).
    virtual void UpdateInternalForces();

    /*
    // Override the ChLoadContainer method for computing the generalized force F term:
    virtual void IntLoadResidual_F(const unsigned int off,  ///< offset in R residual
                                   ChVectorDynamic<>& R,    ///< result: the R residual, R += c*F
                                   const double c           ///< a scaling factor
                                   ) override {
         // reset the internal forces
       // this->GetLoadList().clear();
         // Computes the internal forces
       // this->UpdateInternalForces();
         // Overloading base class, that takes all F vectors from the list of forces and put all them in R
        ChLoadContainer::IntLoadResidual_F(off, R, c);
    };
    */

  private:

      // This is called after Initialize(), it precomputes aux.topology 
      // data structures for the mesh, aux. material data, etc.
    void SetupAuxData();

    std::shared_ptr<ChColorAsset> m_color;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
    double m_height;

    std::vector<ChVector<>> p_vertices_initial;
    std::vector<ChVector<>> p_speeds;
    std::vector<double> p_sinkage;
    std::vector<double> p_sinkage_plastic;
    std::vector<double> p_sinkage_elastic;
    std::vector<double> p_step_plastic_flow;
    std::vector<double> p_kshear; // Janosi-Hanamoto shear accumulator
    std::vector<double> p_area;
    std::vector<double> p_sigma;
    std::vector<double> p_sigma_yeld;
    std::vector<double> p_tau;
    std::vector<int>    p_id_island;
    
    double Bekker_Kphi;
    double Bekker_Kc;
    double Bekker_n;
    double Mohr_cohesion;
    double Mohr_friction;
    double Janosi_shear;
    double elastic_K;

    eChSoilDataPlot plot_type;
    double plot_v_min;
    double plot_v_max;

    ChCoordsys<> plane;

    // aux. topology data
    std::vector<std::set<int>> connected_vertexes;
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
