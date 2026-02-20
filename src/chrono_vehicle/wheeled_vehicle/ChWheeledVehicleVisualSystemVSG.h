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
// Authors: Radu Serban
// =============================================================================
//
// VSG-based visualization for wheeled vehicles.
//
// =============================================================================

#ifndef CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H
#define CH_WHEELED_VEHICLE_VISUAL_SYSTEM_VSG_H

#include "chrono_vehicle/visualization/ChVehicleVisualSystemVSG.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/ChForceElementTire.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_vis
/// @{

/// Customized Chrono::VSG visualization system for wheeled vehicle simulation.
class CH_VEHICLE_API ChWheeledVehicleVisualSystemVSG : public ChVehicleVisualSystemVSG {
  public:
    /// Construct a wheeled vehicle VSG visualization.
    ChWheeledVehicleVisualSystemVSG();

    ~ChWheeledVehicleVisualSystemVSG() {}

    /// Attach a vehicle to this VSG wheeled vehicle visualization system.
    virtual void AttachVehicle(vehicle::ChVehicle* vehicle) override;

    /// Append GUI info specific to wheeled vehicles.
    virtual void AppendGUIStats() override;

    /// Initialize the visualization system.
    virtual void Initialize() override;

    /// Update all VSG scenes with the current state of the associated Chrono systems.
    virtual void Update() override;

    /// Set the visibility for tire-terrain interaction info.
    void SetTireTerrainInfoVisibility(bool vis);

  private:
    ChWheeledVehicle* m_wvehicle;

    bool m_chassis_visible;
    bool m_suspension_visible;
    bool m_steering_visible;
    bool m_wheel_visible;
    bool m_tire_visible;

    std::vector<std::shared_ptr<ChForceElementTire>> m_handling_tires;  ///< references to all handling tires

    bool m_tire_terrain_visible;                     ///< render tire-terrain frames?
    double m_tire_force_scale;                       ///< scaling factor for tire-terrain normal force
    ChVector3d m_tire_terrain_scale;                 ///< scale for tire-terrain frames
    vsg::ref_ptr<vsg::Switch> m_tire_terrain_scene;  ///< VSG scene for tire-terrain frames
};

/// @} vehicle_vis

}  // end namespace vehicle
}  // end namespace chrono

#endif
