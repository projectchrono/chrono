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
// Authors: Antonio Recuero, Bryan Peterson
// =============================================================================
//
// FEA Deformable terrain. Box of terrain composed of 9-node brick elements which
// can capture moderate deformation (no remeshing). Constitutive behavior
//
// =============================================================================

#ifndef FEADEFORMABLE_TERRAIN_H
#define FEADEFORMABLE_TERRAIN_H

#include <set>
#include <string>

#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChSystem.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/ChTerrain.h"

namespace chrono {
namespace vehicle {

class FEADeformableSoil;

/// @addtogroup vehicle_terrain
/// @{

/// FEA Deformable terrain model.
/// This class implements a terrain made up of isoparametric finite elements. It features
/// Drucker-Prager plasticity and capped Drucker-Prager plasticity.
class CH_VEHICLE_API FEADeformableTerrain : public ChTerrain {
  public:
    /// Construct a default DeformableSoil.
    /// The user is responsible for calling various Set methods before Initialize.
    FEADeformableTerrain(ChSystem* system);  ///< [in] pointer to the containing system);

    ~FEADeformableTerrain() {}

    /// Get the terrain height at the specified (x,y) location.
    virtual double GetHeight(double x, double y) const override;

    /// Get the terrain normal at the specified (x,y) location.
    virtual chrono::ChVector<> GetNormal(double x, double y) const override;

    /// Set the properties of the FEA soil.
    /// The meaning of these parameters is described in the paper:
    void SetSoilParametersFEA(double rho,              ///< Soil density
                              double Emod,             ///< Soil modulus of elasticity
                              double nu,               ///< Soil Poisson ratio
                              double kn,               ///< Soil normal contact stiffness coefficient
                              double kt,               ///< Soil tangential contact stiffness coefficient
                              double gn,               ///< Soil normal contact damping coefficient
                              double gt,               ///< Soil tangential contact damping coefficient
                              double yield_stress,     ///< Soil yield stress, for plasticity
                              double hardening_slope,  ///< Soil hardening slope, for plasticity
                              double friction_angle,   ///< Soil internal friction angle
                              double dilatancy_angle   ///< Soil dilatancy angle
                              );

    /// Initialize the terrain system (flat).
    /// This version creates a flat array of points.
    void Initialize(
        const ChVector<>& start_point,                ///< [in] Base point to build terrain box
        const ChVector<>& terrain_dimension,          ///< [in] terrain dimensions in the 3 directions
        const ChVector<int>& terrain_discretization);  ///< [in] Number of finite elements in the 3 directions

    /// Get the underlying FEA mesh.
    std::shared_ptr<fea::ChMesh> GetMesh() const { return m_mesh; }

  private:
    std::shared_ptr<fea::ChMesh> m_mesh;                ///< tire mesh
    std::shared_ptr<ChLoadContainer> m_load_container;  ///< load container (for pressure load)

    double m_rho;  ///< Soil density
    double m_E;    ///< Soil modulus of elasticity
    double m_nu;   ///< Soil Poisson ratio

    double m_kn;  ///< Soil normal contact stiffness
    double m_kt;  ///< Soil tangential contact stiffness
    double m_gn;  ///< Soil normal contact damping
    double m_gt;  ///< Soil tangential contact damping

    double m_yield_stress;     ///< Yield stress for soil plasticity
    double m_hardening_slope;  ///< Hardening slope for soil plasticity
    double m_friction_angle;   ///< Set friction angle for soil plasticity
    double m_dilatancy_angle;  ///< Set dilatancy angle for soil plasticity
};

/// @} vehicle_terrain

}  // end namespace vehicle
}  // end namespace chrono

#endif
