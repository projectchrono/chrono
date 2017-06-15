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
// Irrlicht-based visualization wrapper for tracked vehicles.
// This class extends ChVehicleIrrApp.
//
// =============================================================================

#ifndef CH_TRACKED_VEHICLE_IRRAPP_H
#define CH_TRACKED_VEHICLE_IRRAPP_H

#include "chrono_vehicle/utils/ChVehicleIrrApp.h"
#include "chrono_vehicle/tracked_vehicle/ChTrackedVehicle.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_tracked_utils
/// @{

/// Customized Chrono Irrlicht application for tracked vehicle visualization.
class CH_VEHICLE_API ChTrackedVehicleIrrApp : public ChVehicleIrrApp {
  public:
    /// Construct a tracked vehicle Irrlicht application.
    ChTrackedVehicleIrrApp(
        ChVehicle* vehicle,        ///< pointer to the associated vehicle system
        ChPowertrain* powertrain,  /// pointer to the associated powertrain system
        const wchar_t* title = 0,  ///< window title
        irr::core::dimension2d<irr::u32> dims = irr::core::dimension2d<irr::u32>(1000, 800)  ///< window dimensions
        );

    ~ChTrackedVehicleIrrApp() {}

  private:
    virtual void renderOtherGraphics() override;
    virtual void renderOtherStats(int left, int top) override;
    void renderContactNormals(const std::list<ChTrackContactInfo>& lst, const irr::video::SColor& col);

    ChTrackedVehicle* m_tvehicle;
};

/// @} vehicle_tracked_utils

}  // end namespace vehicle
}  // end namespace chrono

#endif
