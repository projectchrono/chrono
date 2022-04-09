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
// Authors: Radu Serban, Aki Mikkola
// =============================================================================
//
// Template for LuGre tire model
//
// =============================================================================

#ifndef CH_LUGRETIRE_H
#define CH_LUGRETIRE_H

#include <vector>

#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/physics/ChBody.h"

#include "chrono_vehicle/ChTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_tire
/// @{

/// Tire model based on LuGre friction model.
class CH_VEHICLE_API ChLugreTire : public ChForceElementTire {
  public:
    ChLugreTire(const std::string& name);

    virtual ~ChLugreTire() {}

    /// Get the name of the vehicle subsystem template.
    virtual std::string GetTemplateName() const override { return "LugreTire"; }

    /// Add visualization assets for the rigid tire subsystem.
    virtual void AddVisualizationAssets(VisualizationType vis) override;

    /// Remove visualization assets for the rigid tire subsystem.
    virtual void RemoveVisualizationAssets() override;

    /// Get the tire width.
    /// This is just an approximation of a tire width.
    double GetWidth() const;

    /// Report the tire force and moment.
    virtual TerrainForce ReportTireForce(ChTerrain* terrain) const override { return m_tireForce; }

  protected:
    /// Return the number of discs used to model this tire.
    virtual int GetNumDiscs() const = 0;

    /// Return the lateral disc locations.
    /// These locations are relative to the tire center.
    virtual const double* GetDiscLocations() const = 0;

    /// Set the parameters in the LuGre friction model.
    virtual void SetLugreParams() = 0;

    /// Lugre friction model parameters (longitudinal/lateral)
    double m_sigma0[2];  ///<
    double m_sigma1[2];  ///<
    double m_sigma2[2];  ///<
    double m_Fc[2];      ///<
    double m_Fs[2];      ///<
    double m_vs[2];      ///< Stribeck velocity

  //private:
    /// Get the tire force and moment.
    /// This represents the output from this tire system that is passed to the
    /// vehicle system.  Typically, the vehicle subsystem will pass the tire force
    /// to the appropriate suspension subsystem which applies it as an external
    /// force one the wheel body.
    virtual TerrainForce GetTireForce() const override { return m_tireForce; }

    /// Initialize this tire by associating it to the specified wheel.
    virtual void Initialize(std::shared_ptr<ChWheel> wheel) override;

    /// Update the state of this tire system at the current time.
    virtual void Synchronize(double time,              ///< [in] current time
                             const ChTerrain& terrain  ///< [in] reference to the terrain system
                             ) override;

    /// Advance the state of this tire by the specified time step.
    virtual void Advance(double step) override;

    struct DiscContactData {
        bool in_contact;       // true if disc in contact with terrain
        ChCoordsys<> frame;    // contact frame (x: long, y: lat, z: normal)
        ChVector<> vel;        // relative velocity expressed in contact frame
        double normal_force;   // magnitude of normal contact force
        double ode_coef_a[2];  // ODE coefficients:  z' = a + b * z
        double ode_coef_b[2];  //   (longitudinal/lateral)
    };

    struct DiscState {
        double z0;  // longitudinal direction
        double z1;  // lateral direction
    };

    TerrainForce m_tireForce;
    std::vector<DiscContactData> m_data;
    std::vector<DiscState> m_state;

    std::vector<std::shared_ptr<ChCylinderShape>> m_cyl_shapes;  ///< visualization cylinder assets
    std::shared_ptr<ChTexture> m_texture;                        ///< visualization texture asset
};

/// @} vehicle_wheeled_tire

}  // end namespace vehicle
}  // end namespace chrono

#endif
